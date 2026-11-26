"""Ask only our own Qt window to close before resorting to process signals."""
import ctypes as C
import os
import re
import signal
import subprocess
import time


def owned_window(pid, env):
    def prop(*args):
        p = subprocess.run(['xprop', *args], env=env, capture_output=True, text=True,
                           errors='replace', timeout=2)
        return p.stdout if p.returncode == 0 else ''
    for win in re.findall(r'0x[0-9a-fA-F]+', prop('-root', '_NET_CLIENT_LIST')):
        fields = prop('-id', win, '_NET_WM_PID', 'WM_PROTOCOLS', '_NET_WM_WINDOW_TYPE')
        if (re.search(r'_NET_WM_PID\([^)]*\)\s*=\s*'+str(pid)+r'\b', fields)
                and 'WM_DELETE_WINDOW' in fields and '_NET_WM_WINDOW_TYPE_NORMAL' in fields):
            return int(win, 16)
    return None


def request_close(win, env):
    # Xlib's XClientMessageEvent carries five native longs; XEvent is 24 longs.
    class ClientMessage(C.Structure):
        _fields_ = [('type', C.c_int), ('serial', C.c_ulong), ('send', C.c_int),
                    ('display', C.c_void_p), ('window', C.c_ulong), ('message', C.c_ulong),
                    ('format', C.c_int), ('data', C.c_long * 5)]
    class XEvent(C.Union):
        _fields_ = [('client', ClientMessage), ('size', C.c_long * 24)]
    x = C.CDLL('libX11.so.6')
    x.XOpenDisplay.argtypes = [C.c_char_p]; x.XOpenDisplay.restype = C.c_void_p
    x.XInternAtom.argtypes = [C.c_void_p, C.c_char_p, C.c_int]; x.XInternAtom.restype = C.c_ulong
    x.XSendEvent.argtypes = [C.c_void_p, C.c_ulong, C.c_int, C.c_long, C.POINTER(XEvent)]
    x.XFlush.argtypes = x.XCloseDisplay.argtypes = [C.c_void_p]
    display = x.XOpenDisplay(os.fsencode(env['DISPLAY']))
    if not display:
        return False
    try:
        e = XEvent()
        e.client.type, e.client.display, e.client.window, e.client.format = 33, display, win, 32
        e.client.message = x.XInternAtom(display, b'WM_PROTOCOLS', 0)
        e.client.data[0] = x.XInternAtom(display, b'WM_DELETE_WINDOW', 0)
        sent = x.XSendEvent(display, win, 0, 0, C.byref(e))
        x.XFlush(display)
        return bool(sent)
    finally:
        x.XCloseDisplay(display)


def stop_rviz(proc, env):
    if proc.poll() is not None:
        return
    try:
        end = time.monotonic() + 4
        while proc.poll() is None and time.monotonic() < end:
            window = owned_window(proc.pid, env)
            if window and request_close(window, env):
                proc.wait(timeout=10)
                break
            time.sleep(.15)
    except (OSError, subprocess.TimeoutExpired):
        pass
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(8)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(4)
            except subprocess.TimeoutExpired:
                proc.kill(); proc.wait()
    if proc.returncode not in (0, -signal.SIGINT):
        raise RuntimeError(f'RViz 关闭时退出码 {proc.returncode}')
