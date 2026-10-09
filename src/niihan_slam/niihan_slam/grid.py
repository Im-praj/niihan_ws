"""Flat-site occupancy projection: unknown is preserved; occupied endpoints win."""
import numpy as np

def rotation(q):
    x,y,z,w=q;norm=np.linalg.norm(q)
    if norm<1e-9:raise ValueError('Invalid quaternion')
    x,y,z,w=np.array(q)/norm
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)], [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)], [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

class Grid:
    def __init__(self,resolution=.1,size=600):
        self.resolution=resolution;self.size=size;self.origin=-size*resolution/2
        self.free=np.zeros((size,size),dtype=bool);self.occupied=np.zeros_like(self.free)
    def cells(self,xy):return np.floor((np.asarray(xy)-self.origin)/self.resolution).astype(int)
    def update(self,points,origin,ground_z):
        p=np.asarray(points);p=p[np.isfinite(p).all(axis=1)]
        # Ignore elevated overhead structures; not a terrain traversability estimator.
        p=p[(p[:,2]>ground_z+.15)&(p[:,2]<ground_z+1.8)]
        if not len(p):return
        cells=self.cells(p[:,:2]);cells=np.unique(cells,axis=0)
        cells=cells[(cells>=0).all(axis=1)&(cells<self.size).all(axis=1)]
        start=self.cells(np.asarray(origin)[:2]);start=np.clip(start,0,self.size-1)
        # Bound workload with one nearest endpoint per angular bin to avoid clearing
        # through a closer obstacle when a farther ray hits another height.
        delta=cells-start;angles=np.arctan2(delta[:,1],delta[:,0]);bins=np.floor((angles+np.pi)*360/(2*np.pi)).astype(int)
        dist=np.linalg.norm(delta,axis=1);order=np.argsort(dist,kind='stable');_,inds=np.unique(bins[order],return_index=True)
        for end in cells[order[inds]]:
            n=int(np.max(np.abs(end-start)))
            if n:
                ray=np.rint(np.linspace(start,end,n+1)).astype(int)[:-1];self.free[ray[:,1],ray[:,0]]=True
        self.occupied[cells[:,1],cells[:,0]]=True
    def data(self):
        data=np.full(self.free.shape,-1,dtype=np.int8);data[self.free]=0;data[self.occupied]=100;return data

def icp_planar(source,target_tree,initial,iterations=15,max_distance=1.):
    """Align XYZ with planar SE(2); caller rejects weak overlap/RMSE."""
    pose=np.array(initial,dtype=float)
    for _ in range(iterations):
        c,s=np.cos(pose[2]),np.sin(pose[2]);r=np.array([[c,-s],[s,c]])
        xy=source[:,:2]@r.T+pose[:2];p=np.column_stack((xy,source[:,2]))
        distance,index=target_tree.query(p);mask=distance<max_distance
        if mask.sum()<30:return pose,float('inf'),float(mask.mean())
        a=xy[mask];b=target_tree.data[index[mask],:2];ac=a.mean(axis=0);bc=b.mean(axis=0)
        u,_,vt=np.linalg.svd((a-ac).T@(b-bc));rr=vt.T@u.T
        if np.linalg.det(rr)<0:vt[-1]*=-1;rr=vt.T@u.T
        dyaw=np.arctan2(rr[1,0],rr[0,0]);translation=bc-ac@rr.T
        pose[:2]=pose[:2]@rr.T+translation;pose[2]+=dyaw
        if np.linalg.norm(translation)<1e-4 and abs(dyaw)<1e-4:break
    c,s=np.cos(pose[2]),np.sin(pose[2]);p=source.copy();p[:,:2]=source[:,:2]@np.array([[c,-s],[s,c]]).T+pose[:2]
    d,_=target_tree.query(p);ok=d<max_distance
    return pose,float(np.sqrt(np.mean(d[ok]**2))) if ok.any() else float('inf'),float(ok.mean())
