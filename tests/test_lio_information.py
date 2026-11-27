"""Check diagnostic spectral and variance decomposition against independent NumPy calculations."""
import csv
import os
from pathlib import Path
import runpy
import subprocess
import tempfile

import numpy as np


def test_information():
    package=Path(__file__).resolve().parents[1]/"src"/"fast_livo"
    fixture=r'''
#include "lio_information.hpp"
int main() {
  using namespace lio_information;
  const double c=0.8,s=0.6;
  Eigen::Matrix3d R; R<<c,-s,0,s,c,0,0,0,1;
  const Vec3 gravity(-1,2,-8), origin(0.3,-0.2,0.5);
  Sample audit; audit.frame(R,gravity,origin);
  Mat6 info=Mat6::Zero();
  for(int i=0; i<15; ++i){
    Vec3 p(i%3-1.25,i%4+0.4,i%5-1.8);
    Vec3 normal=Vec3::Unit(i%3);
    Vec3 arm=p.cross(R.transpose()*normal);
    double plane=(i+1)*0.0002, point=(1+i%4)*0.0003, w=1.0/(0.001+plane+point);
    Eigen::Matrix<double,6,1> h; h<<arm,normal;
    info+=w*h*h.transpose();
    audit.add(R*p+origin,normal,arm,p.squaredNorm(),plane,point,w);
  }
  audit.write(123.25,3,info);
}
'''
    with tempfile.TemporaryDirectory() as work:
        d=Path(work)
        (d/"fixture.cpp").write_text("#include <Eigen/Geometry>\n"+fixture,encoding="utf-8")
        subprocess.run(["g++","-std=c++17","-O1","-I/usr/include/eigen3","-I",str(package/"include"),
                        str(d/"fixture.cpp"),str(package/"src"/"lio_information.cpp"),"-o",str(d/"fixture")],check=True)
        subprocess.run([str(d/"fixture")],env=dict(os.environ,FAST_LIVO_DIAGNOSTICS_DIR=work),check=True)
        with (d/"lio_information_6d.csv").open(newline="") as f:
            rec=next(csv.DictReader(f))
        with (d/"measurement_weight_sources.csv").open(newline="") as f:
            terms=list(csv.DictReader(f))
        summary_script=package.parents[1]/"tools"/"summarize_lio_information.py"
        summary=runpy.run_path(str(summary_script))["summarize"](d)
        assert summary["valid_samples"]==1 and summary["classes"]["all"]["count"]==15
    r=np.array([[.8,-.6,0],[.6,.8,0],[0,0,1]])
    axis=np.array([1.,-2.,8.]); axis/=np.linalg.norm(axis)
    p=np.array([[i%3-1.25,i%4+0.4,i%5-1.8] for i in range(15)])
    n=np.eye(3)[np.arange(15)%3]
    scale=np.sqrt(np.mean(np.sum(p*p,axis=1)))
    variance=.001+(np.arange(15)+1)*.0002+(1+np.arange(15)%4)*.0003
    # Construct geometry directly in world coordinates from rotated lever arms.
    h=np.column_stack([np.cross(p@r.T,n)/scale,n])
    info=h.T@(h/variance[:,None])
    logged=np.zeros((6,6))
    for i in range(6):
        for j in range(i,6):
            logged[i,j]=logged[j,i]=float(rec[f"h{i}{j}"])
    np.testing.assert_allclose(logged,info,rtol=1e-10,atol=1e-10)
    vals=np.array([float(rec[f"lambda{i}"]) for i in range(6)])
    np.testing.assert_allclose(vals,np.linalg.eigvalsh(info))
    weak=np.array([float(rec["weak_"+k]) for k in ("rx","ry","rz","tx","ty","tz")])
    np.testing.assert_allclose(info@weak,vals[0]*weak,atol=1e-9)
    np.testing.assert_allclose(float(rec["world_tup_information"]),axis@info[3:,3:]@axis)
    np.testing.assert_allclose(float(rec["weak_translation_up"]),weak[3:]@axis)
    np.testing.assert_allclose(float(rec["rms_arm_m"]),scale)
    assert int(terms[0]["count"])==sum(int(q["count"]) for q in terms[1:])==15
    for q in terms:
        np.testing.assert_allclose(float(q["inverse_weight_sum_m2"]),
                sum(float(q[k]) for k in ("sigma_plane_sum_m2","sigma_point_sum_m2","fixed_variance_sum_m2")))
    np.testing.assert_allclose(float(terms[0]["weight_sum"]),np.sum(1/variance))
    np.testing.assert_allclose(float(terms[0]["vertical_translation_information"]),axis@info[3:,3:]@axis)
    np.testing.assert_allclose(float(terms[0]["yaw_rotation_information"])/scale**2,axis@info[:3,:3]@axis)


if __name__=="__main__":
    test_information()
    print("PASS independent world Jacobian, six eigenvalues, weakest direction, gravity projections and weight ledger")
