"""Full-cloud projection in bounded memory; the source PCDs retain every finite point."""
from PIL import Image, ImageDraw
import numpy as np


def points(path):
    with path.open("rb") as f:
        while f.readline().strip() != b"DATA binary":
            if f.tell()>8192:
                raise ValueError("bad map header")
        while True:
            a=np.fromfile(f,dtype="<f4",count=4*200000)
            if not a.size:
                return
            yield a.reshape(-1,4)[:,:3]


def preview(out, before, after):
    size=(840,470)
    image=Image.new("RGB",(2*size[0],2*(size[1]+26)),(247,248,248))
    draw=ImageDraw.Draw(image)
    bounds={}
    for row, ax in enumerate(((0,1),(0,2))):
        # Same metric scale, origin and crop in each before/after pair.
        trajectory=np.concatenate([before[:,:3,3],after[:,:3,3]])[:,ax]
        lo,hi=trajectory.min(0)-12,trajectory.max(0)+12
        scale=min((size[0]-8)/(hi[0]-lo[0]),(size[1]-8)/(hi[1]-lo[1]))
        center=(hi+lo)/2
        def pixels(arr):
            a=(arr-center)*scale
            return np.column_stack([size[0]/2+a[:,0],size[1]/2-a[:,1]]).astype(int)
        bounds[str(ax)]={"scale_px_per_m":float(scale),"center_m":center.tolist()}
        for col,(name,path) in enumerate((("before",before),("after",after))):
            hist=np.zeros(size[::-1],np.uint32)
            for chunk in points(out/f"map_{name}.pcd"):
                p=pixels(chunk[:,ax])
                p=p[(p[:,0]>=0)&(p[:,0]<size[0])&(p[:,1]>=0)&(p[:,1]<size[1])]
                np.add.at(hist,(p[:,1],p[:,0]),1)
            ink=np.minimum(np.log1p(hist)/np.log(30),1)
            img=(249-ink[:,:,None]*np.array([192,175,156])).astype("uint8")
            tile=Image.fromarray(img)
            layer=ImageDraw.Draw(tile)
            path_px=[tuple(v) for v in pixels(path[:,:3,3][:,ax])]
            layer.line(path_px,fill=(199,55,28) if col==0 else (0,106,86),width=2)
            a,b=path_px[0],path_px[-1]
            layer.ellipse((a[0]-4,a[1]-4,a[0]+4,a[1]+4),outline=(10,40,220),width=2)
            layer.rectangle((b[0]-4,b[1]-4,b[0]+4,b[1]+4),outline=(150,0,150),width=2)
            x,y=col*size[0],row*(size[1]+26)
            draw.text((x+10,y+5),f"{name.upper()}    {'XY top' if row==0 else 'XZ side'}    map_gravity / m",fill=(25,35,40))
            image.paste(tile,(x,y+26))
    image.save(out/"map_comparison.png")
    return bounds
