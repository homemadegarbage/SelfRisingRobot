# 作者：同济子豪兄 2026-10-26 | RoBoBo V3.9
"""Read-only Wi-Fi pose mirror. macOS: .venv/bin/mjpython live_robot_viewer.py"""
import argparse,json,socket,time,math,threading,sys,ctypes
from pathlib import Path
import numpy as np
import mujoco
from viewer_theme import load_dark_model

def parse_packet(raw):
    p=json.loads(raw)
    if p.get('schema')!='robobo.pose.v1':raise ValueError('unknown schema')
    q=np.asarray(p['q'],dtype=float);t=np.asarray(p['targets'],dtype=float)
    if q.shape!=(4,) or t.shape!=(2,) or not np.isfinite(q).all() or not np.isfinite(t).all():raise ValueError('invalid values')
    n=np.linalg.norm(q)
    if not .9<n<1.1:raise ValueError('invalid quaternion')
    if np.any(np.abs(t)>np.array([np.pi/2,2*np.pi/3])+.001):raise ValueError('invalid target')
    if type(p['seq']) is not int or type(p['boot']) is not int:raise ValueError('invalid sequence')
    if type(p['enabled']) is not bool or type(p['valid']) is not bool:raise ValueError('invalid flags')
    return p,q/n,t

class PoseMirror:
    def __init__(self,xml):
        self.model=load_dark_model(xml);self.data=mujoco.MjData(self.model)
        ids=[mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,n) for n in ['servo1_joint','servo2_joint']]
        if min(ids)<0:raise ValueError('XML must contain both servo joints')
        self.qids=self.model.jnt_qposadr[ids];self.vertices=[]
        for g in range(self.model.ngeom):
            if self.model.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH:
                m=self.model.geom_dataid[g];start=self.model.mesh_vertadr[m];count=self.model.mesh_vertnum[m]
                self.vertices.append((g,self.model.mesh_vert[start:start+count]))
    def update(self,q,t):
        self.data.qpos[:3]=0;self.data.qpos[3:7]=q;self.data.qpos[self.qids]=t
        self.data.qvel[:]=0;self.data.ctrl[:]=t;mujoco.mj_forward(self.model,self.data)
        # Display-only grounding, NOT measured height or world translation.
        low=min(np.min(v@self.data.geom_xmat[g].reshape(3,3)[2]+self.data.geom_xpos[g,2]) for g,v in self.vertices)
        self.data.qpos[2]=.002-low;mujoco.mj_forward(self.model,self.data)

def blend_orientation(current,target,alpha):
    # Shortest quaternion arc, normalized; handles equivalent opposite signs.
    if np.dot(current,target)<0:target=-target
    q=(1-alpha)*current+alpha*target
    return q/np.linalg.norm(q)

class LatestPoseReceiver:
    """Drain UDP independently of drawing; retain one latest sample, never a playback queue."""
    def __init__(self,host,port):
        self.host=host;self.latest=None;self.lock=threading.Lock();self.stop=threading.Event()
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.bind(('',port));self.sock.settimeout(.05)
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def run(self):
        boot=None;seq=-1;subscribe=0
        while not self.stop.is_set():
            now=time.monotonic()
            if now-subscribe>=1:
                try:self.sock.sendto(b'ROBOBO_SUBSCRIBE_V1',(self.host,4242))
                except OSError:pass # Wi-Fi may still be connecting.
                subscribe=now
            try:raw,addr=self.sock.recvfrom(2048)
            except socket.timeout:continue
            except OSError:break
            if addr[0]!=self.host:continue
            try:packet,q,t=parse_packet(raw)
            except (ValueError,KeyError,TypeError,OverflowError):continue
            if packet['boot']==boot and packet['seq']<=seq:continue
            boot=packet['boot'];seq=packet['seq']
            with self.lock:self.latest=(packet,q,t,time.monotonic())
    def snapshot(self):
        with self.lock:return self.latest
    def close(self):
        self.stop.set();self.thread.join(timeout=.2);self.sock.close()

def display_available():
    if sys.platform!='darwin':return True
    cg=ctypes.CDLL('/System/Library/Frameworks/CoreGraphics.framework/CoreGraphics')
    count=ctypes.c_uint32()
    return cg.CGGetActiveDisplayList(0,None,ctypes.byref(count))==0 and count.value>0

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--host',default='192.168.42.1',help='Only accept UDP from this robot IP');p.add_argument('--port',type=int,default=4242);p.add_argument('--xml',type=Path,default=Path(__file__).with_name('robo_s3r.xml'));p.add_argument('--check',action='store_true',help='Headless packet/model self-check');p.add_argument('--no-smoothing',action='store_true',help='Display raw latest sample for diagnostics');p.add_argument('--duration',type=float,default=0,help='Optional viewer runtime in seconds');a=p.parse_args()
    mirror=PoseMirror(a.xml)
    if a.check:
        for q in [[1,0,0,0],[.70710678,.70710678,0,0],[0,1,0,0]]:
            raw=json.dumps(dict(schema='robobo.pose.v1',q=q,targets=[.3,-.4],seq=1,boot=1,enabled=True,valid=True));_,qn,t=parse_packet(raw);mirror.update(qn,t);assert np.isfinite(mirror.data.geom_xpos).all();assert np.allclose(mirror.data.qpos[mirror.qids],t)
        for bad in [b'{}',b'{',json.dumps(dict(schema='robobo.pose.v1',q=[0]*4,targets=[0,0],seq=1,boot=1,enabled=True,valid=True)).encode()]:
            try:parse_packet(bad)
            except (ValueError,KeyError,TypeError):pass
            else:raise AssertionError('invalid packet accepted')
        print('Packet validation and MuJoCo pose mapping checks passed');return
    if not display_available():
        print('Mac 没有活动显示器。请唤醒屏幕、进入桌面后重新双击启动。');return
    import mujoco.viewer
    try:receiver=LatestPoseReceiver(a.host,a.port)
    except OSError as e:
        print(f'无法监听 UDP {a.port}，请关闭旧查看器或测速程序后重试：{e}');return
    opened=time.monotonic();last=0;status='WAITING: connect RoBoBo Wi-Fi and enable web switch';last_print=''
    displayQ=np.array([1.,0,0,0]);targetQ=displayQ.copy();displayT=np.zeros(2);targetT=displayT.copy();lastFrame=time.monotonic();havePose=False
    mirror.update(displayQ,displayT)
    print('Read-only viewer. No servo commands. Allow incoming UDP if macOS asks. Ctrl+C / close window to exit.')
    try:
        with mujoco.viewer.launch_passive(mirror.model,mirror.data) as viewer:
            viewer.cam.distance=.55;viewer.cam.elevation=-30
            while viewer.is_running() and (not a.duration or time.monotonic()-opened<a.duration):
                start=time.monotonic()
                sample=receiver.snapshot()
                if sample is not None:
                    packet,q,t,last=sample
                    status='LIVE' if packet['enabled'] and packet['valid'] else 'OFF / IMU NOT READY'
                    if status=='LIVE':
                        targetQ=q;targetT=t
                        if not havePose:displayQ=q.copy();displayT=t.copy();havePose=True
                now=time.monotonic();age=now-last
                if havePose and age<1 and status=='LIVE':
                    # Only 5 ms filtering, ~0.6 ms additional lag at 60 fps;
                    # no delayed interpolation buffer or invented motion prediction.
                    alpha=1 if a.no_smoothing else 1-math.exp(-min(.1,now-lastFrame)/.005)
                    displayQ=blend_orientation(displayQ,targetQ,alpha);displayT+=alpha*(targetT-displayT)
                    with viewer.lock():mirror.update(displayQ,displayT)
                lastFrame=now
                shown='STALE: no packets (>1s); pose frozen' if last and age>1 else status
                if shown!=last_print:print(shown);last_print=shown
                viewer.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_TOPLEFT,'RoBoBo Wi-Fi pose mirror',shown+'\nIMU orientation + commanded servo angles\nXY fixed / height display-only / yaw may drift')])
                with viewer.lock():viewer.cam.lookat[:]=mirror.data.subtree_com[1]
                viewer.sync();time.sleep(max(0,1/60-(time.monotonic()-start)))
    except KeyboardInterrupt:pass
    finally:receiver.close()
if __name__=='__main__':main()
