"""Create the published plots. All displayed times are labeled by their clock."""
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

def draw(out,raw,cm,em):
    available={f.name for f in font_manager.fontManager.ttflist}
    fonts=[s for s in ("Microsoft YaHei","Noto Sans CJK SC","SimHei","SimSun") if s in available]
    plt.rcParams.update({"font.family":(fonts or ["DejaVu Sans"]),"axes.unicode_minus":False,"axes.spines.top":False,"axes.spines.right":False,"font.size":10})
    def finish(fig,fn):
        fig.tight_layout(pad=1.3)
        fig.savefig(out/fn,dpi=165,bbox_inches="tight")
        plt.close(fig)
    colors=["#13516b","#b54636","#46826c","#9c6a20"]
    cam=cm["camera"]; events=pd.read_csv(out/"camera_anomalies.csv"); fig,axs=plt.subplots(1,2,figsize=(11,4.1),gridspec_kw={"width_ratios":[1.35,1]})
    x=np.arange(len(events)); bw=.25
    for j,(col,label) in enumerate((("header_dt_ms","图像共享头"),("sdk_get_dt_ms","主机取帧"),("device_dt_ms","设备帧时钟"))):
        axs[0].bar(x+(j-1)*bw,events[col],width=bw,color=colors[j],label=label)
    axs[0].set_xticks(x,events.frame.astype(str),rotation=35); axs[0].set(ylabel="相邻帧间隔（ms）",xlabel="相机帧号",title="共享头异常的九个间隔"); axs[0].legend(fontsize=8)
    names=["图像","相机信息","雷达","IMU"]
    topics=["/hik_camera/image_timeshare","/hik_camera/camera_info_timeshare","/livox/lidar","/livox/imu"]
    tt={(v["topic"],v["clock"]):v for v in cm["streams"]}
    for j,k in enumerate(("source","record")):
        vals=[tt[(n,k)]["p99"]/(5 if i==3 else 100) for i,n in enumerate(topics)]
        axs[1].bar(np.arange(4)+(j-.5)*.34,vals,.34,label=["传感器消息头","Bag 记录时间"][j],color=colors[j])
    axs[1].axhline(1,lw=1,color="#888",ls="--");axs[1].set_xticks(np.arange(4),names);axs[1].set(ylabel="P99 间隔 / 名义周期",title="同一流的两种时钟");axs[1].legend(fontsize=8)
    finish(fig,"camera_timing.png")
    a=pd.read_csv(out/"estimator_60s.csv"); s=pd.read_csv(out/"static_updates.csv"); fig,axs=plt.subplots(2,2,figsize=(11.3,7))
    t=(a.start_s+a.end_open_s)/120
    for ax,col,mul,label in ((axs[0,0],"mean_abs_plane_m",1000,"逐窗点面平均绝对残差（mm）"),(axs[0,1],"vio_position_step_m",1000,"同戳视觉位置修正（mm）"),(axs[1,0],"speed_m_s",1,"LIO 后验速度模（m/s）")):
        ax.plot(t,a[col+"_p50"]*mul,color=colors[0],label="每 60 s 的 P50")
        ax.fill_between(t,a[col+"_p50"]*mul,a[col+"_p95"]*mul,color=colors[0],alpha=.17,label="P50 至 P95")
        ax.set(xlabel="相对首个 LIO 状态（min）",ylabel=label); ax.legend(fontsize=8); ax.grid(alpha=.18)
    ax=axs[1,1]
    for confirmed,g in s.groupby("log_match"):
        ax.scatter(g.t_s/60,g.innovation_norm_rad_s,s=17,color=colors[int(confirmed)],label=("状态与 IMU 重算" if not confirmed else "重算与现场日志吻合"))
    ax.set_yscale("log");ax.axvline(em["markers"]["MOVE_START"]["nearest_sensor_cut_s"]/60,color="#777",ls="--",label="MOVE_START 近似时刻");ax.set(xlabel="相对首个 LIO 状态（min）",ylabel="零角速创新模（rad/s）");ax.legend(fontsize=7.8);ax.grid(alpha=.18)
    finish(fig,"estimator_timeline.png")
    rss=pd.read_csv(out/"resources_samples.csv"); vc=pd.read_csv(out/"visual_cache_60s.csv"); pr=pd.read_csv(out/"processing_60s.csv")
    fig,axs=plt.subplots(2,2,figsize=(11.3,7))
    ax=axs[0,0]
    for col,name,co in (("rss_algorithm_mib","前端",colors[0]),("rss_camera_mib","相机",colors[1]),("rss_archive_mib","归档",colors[2]),("rss_loop_backend_mib","在线后端",colors[3])):
        ax.plot(rss.elapsed_s/60,rss[col],label=name,color=co,lw=1.25)
    ax.set(xlabel="会话启动后（min）",ylabel="进程 RSS（MiB）"); ax.legend(fontsize=8,ncol=2)
    ax=axs[0,1]; t=(vc.start_s+vc.end_open_s)/120
    ax.plot(t,vc.image_mib_max,label="缓存图像",color=colors[0]); ax.plot(t,vc.protected_mib_max,label="受保护图像",color=colors[1]);ax.set(xlabel="相对首个 LIO 状态（min）",ylabel="60 s 内最大采样值（MiB）");ax.legend(fontsize=8)
    ax=axs[1,0]; tp=(pr.start_s+pr.end_open_s)/120
    ax.plot(tp,pr.processing_ms_p50,label="P50",color=colors[0]);ax.plot(tp,pr.processing_ms_p95,label="P95",color=colors[1]);ax.plot(tp,pr.processing_ms_max,label="最大",color=colors[2],lw=1)
    ax.axhline(100,color="#777",ls="--",lw=1);ax.set(xlabel="相对首个 LIO 状态（min）",ylabel="采样处理时间（ms）");ax.legend(fontsize=8)
    ax=axs[1,1];ax.plot(rss.elapsed_s/60,rss.free_disk_gib,label="数据盘可用空间",color=colors[0]);ax.plot(rss.elapsed_s/60,rss.available_gib,label="系统可用内存",color=colors[1]);ax.set(xlabel="会话启动后（min）",ylabel="GiB");ax.legend(fontsize=8)
    for ax in axs.flat: ax.grid(alpha=.18)
    finish(fig,"runtime_resources.png")
    # Message counts and interval tails for every original topic.
    short={"I":"图像","C":"CameraInfo","M":"相机元信息","E":"相机事件","T":"相机锁存","L":"雷达","U":"IMU"}
    from bag_extract import NAMES
    order=["I","C","M","E","T","L","U"]; yy=np.arange(len(order))
    name_by_key={v:k for k,v in NAMES.items()}
    tt={(v["topic"],v["clock"]):v for v in cm["streams"]}
    fig,axs=plt.subplots(1,3,figsize=(12.5,4.6),sharey=True,gridspec_kw={"width_ratios":[1,1.15,1.15]})
    counts=[cm["topic_counts"][name_by_key[k]] for k in order]; ax=axs[0]; ax.barh(yy,counts,color=colors[0],alpha=.85)
    ax.set_xscale("log"); ax.set_xlim(700,max(counts)*6); ax.set(title="逐片核对后的记录数",xlabel="消息条数（对数轴）");ax.set_yticks(yy,[short[k] for k in order]);ax.invert_yaxis()
    for y,count in zip(yy,counts): ax.text(count*1.08,y,f"{count:,}",va="center",fontsize=8)
    for ax,clock,label in ((axs[1],"record","Bag 记录间隔"),(axs[2],"source","源字段间隔")):
        for nm,mk,col in zip(("p50","p95","p99","max"),("o","s","^","|"),colors):
            vals=[tt[(name_by_key[k],clock)][nm] for k in order]
            ax.scatter(vals,yy,marker=mk,color=col,s=32,label=nm.upper(),zorder=2)
        for y,k in enumerate(order):
            v=tt[(name_by_key[k],clock)];ax.plot([v["p50"],v["max"]],[y,y],color="#bbbdbe",lw=.9,zorder=1)
        ax.set_xscale("log");ax.set(xlabel="相邻间隔（ms，对数轴）",title=label);ax.grid(axis="x",alpha=.19)
        ax.legend(loc="lower center",bbox_to_anchor=(.5,-.33),ncol=2,fontsize=8,frameon=False)
    finish(fig,"seven_topic_timing.png")
    rd=pd.read_csv(out/"attitude_elevation_1s.csv")
    fig,axs=plt.subplots(3,2,figsize=(12.2,9.2)); mins=rd.t_mid_s/60
    axs[0,0].plot(mins,rd.roll_map_deg,color=colors[0],lw=.8,label="横滚")
    axs[0,0].plot(mins,rd.pitch_map_deg,color=colors[1],lw=.8,label="俯仰");axs[0,0].set(ylabel="IMU 姿态（°）");axs[0,0].legend(fontsize=8,ncol=2)
    axs[0,1].plot(mins,rd.yaw_map_unwrapped_deg,color=colors[2],lw=.85);axs[0,1].set(ylabel="连续展开的偏航（°）")
    axs[1,0].plot(mins,rd.speed_state_m_s,color=colors[0],lw=.8,label="速度模");axs[1,0].plot(mins,rd.vertical_velocity_state_m_s,color=colors[1],lw=.7,label="竖直分量");axs[1,0].legend(fontsize=8);axs[1,0].set(ylabel="状态速度（m/s）")
    axs[1,1].plot(mins,rd.z_rel_start_m,color=colors[0],lw=1,label="距首点净高度");axs[1,1].set(ylabel="重力系相对高度（m）")
    axs[2,0].plot(mins,rd.cumulative_up_m,color=colors[2],label="逐秒累计上行");axs[2,0].plot(mins,rd.cumulative_down_m,color=colors[1],label="逐秒累计下行");axs[2,0].legend(fontsize=8);axs[2,0].set(ylabel="累计高度变化（m）")
    motion=rd[rd.move].copy()
    ax=axs[2,1]
    ax.scatter(motion.delta_pitch_from_start_deg,motion.delta_z_m,s=6,alpha=.15,color=colors[0],label="1 s 运动窗",linewidths=0)
    motion["pitch_bin"]=pd.qcut(motion.delta_pitch_from_start_deg,16,labels=False,duplicates="drop")
    groups=motion.groupby("pitch_bin")
    x=groups.delta_pitch_from_start_deg.median()
    y=groups.delta_z_m
    ax.fill_between(x,y.quantile(.1),y.quantile(.9),color=colors[1],alpha=.13,label="等样本组 P10–P90")
    ax.plot(x,y.median(),color=colors[1],lw=2,label="等样本组中位数")
    ax.set_yscale("symlog",linthresh=.03,linscale=.75)
    ax.set_yticks([-.6,-.1,-.03,0,.03,.1,.6],["−0.6","−0.1","−0.03","0","0.03","0.1","0.6"])
    ax.set(xlabel="相对起始静止窗的俯仰变化（°）",ylabel="每秒高度净变化（m，对称对数轴）")
    ax.legend(fontsize=7.5);ax.grid(alpha=.15)
    for ax in axs.flat[:-1]:
        ax.set_xlabel("相对首个 LIO 状态（min）"); ax.grid(alpha=.2)
    finish(fig,"attitude_speed_elevation.png")
