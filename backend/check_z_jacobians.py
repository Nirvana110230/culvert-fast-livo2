"""Check source-origin loop and shared-plane Jacobians in GTSAM right coordinates."""
import json
import gtsam
import numpy as np
from loop_edges import normalized_right_jacobian
from shared_planes import point_factor


def derivative(fun, dim, eps=1e-6):
    delta=eps*np.eye(dim)
    return np.column_stack([(fun(dx)-fun(-dx))/(2*eps) for dx in delta])


def run():
    pose=gtsam.Pose3(gtsam.Rot3.RzRyRx(.42,-.18,.53),np.array([3.1,-2.,1.4]))
    p=np.array([[1.5,3.2,-.8],[2.,2.5,-1.],[-.8,.4,.3]])
    n=np.array([[.3,.4,.5],[-.23,.73,.13],[.62,-.15,.3]])
    n/=np.linalg.norm(n,axis=1)[:,None]
    R=pose.rotation().matrix()
    radius=np.sqrt(np.mean(np.sum(p*p,axis=1)))
    scale=np.array([radius]*3+[1.]*3)
    J=normalized_right_jacobian(p,n,R,radius)
    def error(dx):
        T=pose.compose(gtsam.Pose3.Expmap(dx/scale))
        return np.sum(n*(p@T.rotation().matrix().T+T.translation()),axis=1)
    y=p@R.T+pose.translation()
    mu=y.mean(0)
    centered=np.cross(y-mu,n)@R
    arm=np.cross(np.broadcast_to(mu-pose.translation(),n.shape),n)@R
    assert np.max(abs(arm))>.1
    center_error=np.max(abs((centered+arm)/radius-J[:,:3]))
    jerr=np.max(abs(J-derivative(error,6)))
    key=gtsam.symbol('l',3)
    group=dict(center=np.array([2.,1.,-.6]),normal=np.array([0.,0.,1.]),axes=np.eye(3)[:,:2])
    factor=point_factor(0,key,p,group,gtsam.noiseModel.Isotropic.Sigma(3,1.))
    beta=np.array([.1,-.21,.4])
    def values(dx,dp):
        val=gtsam.Values()
        val.insert(0,pose.compose(gtsam.Pose3.Expmap(dx)))
        val.insert(key,beta+dp)
        return val
    def residual(dx,dp):
        return factor.unwhitenedError(values(dx,dp))
    z=np.zeros(6)
    linear=factor.linearize(values(z,z[:3]))
    A=linear.getA().copy()
    numeric=np.column_stack((derivative(lambda dx:residual(dx,z[:3]),6),
                             derivative(lambda dp:residual(z,dp),3)))
    perr=np.max(abs(A-numeric))
    result=dict(loop_right_max_abs=float(jerr),origin_transport_max_abs=float(center_error),
                plane_pose_and_landmark_max_abs=float(perr))
    assert max(result.values())<1e-7,result
    print(json.dumps(result))


if __name__=="__main__":
    run()
