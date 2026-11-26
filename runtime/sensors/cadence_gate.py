"""Transport readiness only: reject a burst of buffered startup messages.

This compares elapsed durations within each clock, never pairs camera and lidar.
"""
def cadence_ready(streams,now_mono_ns):
    rules={'lidar':(12,200000000),'imu':(200,20000000),'image':(12,250000000)}
    details={}
    for name,(count,max_step) in rules.items():
        rows=streams.get(name,[])[-count:]
        if len(rows)<count:
            details[name]={'ready':False,'reason':'insufficient_rows'};continue
        source=[int(r['header_ns']) for r in rows]
        host=[int(r['observer_monotonic_ns']) for r in rows]
        source_steps=[b-a for a,b in zip(source,source[1:])]
        host_steps=[b-a for a,b in zip(host,host[1:])]
        span=source[-1]-source[0];host_span=host[-1]-host[0];age=now_mono_ns-host[-1]
        ok=(all(0<x<=max_step for x in source_steps) and all(x>0 for x in host_steps)
            and host_span>=800000000 and abs(span-host_span)<=100000000 and 0<=age<=1500000000)
        details[name]={'ready':ok,'source_span_ns':span,'host_span_ns':host_span,'last_row_age_ns':age}
    return all(x['ready'] for x in details.values()),details
