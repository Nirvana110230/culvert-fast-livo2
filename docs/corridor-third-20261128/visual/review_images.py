"""Make a labeled review sheet for each archived stage and channel, plus report images."""
from argparse import ArgumentParser
from pathlib import Path
import csv
from PIL import Image, ImageDraw, ImageFont, ImageOps


def main():
    parser=ArgumentParser(description=__doc__)
    parser.add_argument("originals",type=Path)
    parser.add_argument("output",type=Path)
    args=parser.parse_args()
    out=args.output
    out.mkdir(parents=True,exist_ok=True)
    sheets=out/"sheets"
    sheets.mkdir(exist_ok=True)
    font=ImageFont.truetype("C:/Windows/Fonts/msyh.ttc",20)
    records=[]
    for stage,channel,tag in [(s,ch,t) for s in ("pgo_before","pgo_after","structure_before","structure_after")
                             for ch,t in (("几何","geometry"),("彩色","rgb"))]:
        paths=sorted(args.originals.glob(f"*/{stage}{channel}*.png"))
        layers=[]
        for path in paths:
            with Image.open(path) as img:
                img.load()
                view=path.stem.split("_")[-1]
                records.append((stage,channel,view,path.relative_to(args.originals).as_posix(),img.width,img.height,path.stat().st_size))
                canvas=Image.new("RGB",(750,445),"white")
                thumb=ImageOps.contain(img.convert("RGB"),(746,407))
                canvas.paste(thumb,((750-thumb.width)//2,34))
                ImageDraw.Draw(canvas).text((9,5),f"{stage}  {channel}  {view}",fill="black",font=font)
                layers.append(canvas)
                if stage!="structure_before" and ((tag=="geometry" and ("局部1" in view or view=="XZ视角")) or
                                                  (tag=="rgb" and view=="Front视角")):
                    label="detail" if "局部" in view else "xz" if view=="XZ视角" else "front"
                    ImageOps.contain(img.convert("RGB"),(1460,980)).save(out/f"{stage}_{tag}_{label}.jpg",quality=91,subsampling=0)
        sheet=Image.new("RGB",(1500,445*((len(layers)+1)//2)),(242,243,243))
        for k,layer in enumerate(layers):
            sheet.paste(layer,(750*(k%2),445*(k//2)))
        sheet.save(sheets/f"{stage}_{tag}.jpg",quality=90)
    with (out/"screenshots.csv").open("w",encoding="utf-8-sig",newline="") as f:
        w=csv.writer(f);w.writerow(["stage","channel","view","source_png","width","height","bytes"]);w.writerows(records)
    for name,channel,view in [("xz_strip","几何","XZ视角"),("rgb_detail_strip","彩色","俯视角_局部")]:
        canvas=Image.new("RGB",(1100,3*355),"white")
        for k,(stage,title) in enumerate(zip(("pgo_before","pgo_after","structure_after"),("A  前端","B  全程 PGO","C  联合优化"))):
            path=next(args.originals.glob(f"*/{stage}{channel}地图_{view}*.png"))
            with Image.open(path) as img:
                thumb=ImageOps.contain(img.convert("RGB"),(1020,330))
                canvas.paste(thumb,((canvas.width-thumb.width)//2,355*k+24))
            ImageDraw.Draw(canvas).text((16,355*k+1),title,fill="black",font=font)
        canvas.save(out/f"{name}.jpg",quality=91)


if __name__=="__main__":
    main()
