import sys
from PIL import Image
import numpy as np
img=np.array(Image.open(sys.argv[1]).convert('RGB')).astype(int)
H,W,_=img.shape
# 只看右侧地图面板,避开顶部工具栏
sub=img[60:770,780:W]
white=(sub[:,:,0]>245)&(sub[:,:,1]>245)&(sub[:,:,2]>245)
cols=np.where(white.sum(axis=0)>60)[0]; rows=np.where(white.sum(axis=1)>60)[0]
mx0,mx1,my0,my1=cols.min(),cols.max(),rows.min(),rows.max()
Wm,Hm=mx1-mx0,my1-my0
print(f"地图矩形 {Wm}x{Hm} 纵横比={Wm/Hm:.2f} (数据 200x140 -> 1.43)")
mapreg=img[my0+60:my1+60+1, mx0+780:mx1+780+1]
blk=(mapreg[:,:,0]<10)&(mapreg[:,:,1]<10)&(mapreg[:,:,2]<10)
# 用连通域分开两个角块
from scipy import ndimage
lab,n=ndimage.label(blk)
print("黑色连通块数量:",n)
sizes=ndimage.sum(blk,lab,range(1,n+1))
for idx in np.argsort(sizes)[::-1][:4]:
    ys,xs=np.where(lab==idx+1)
    print(f"  块{idx+1}: 面积{int(sizes[idx])} x[{100*xs.min()/Wm:.0f}%,{100*xs.max()/Wm:.0f}%] y[{100*ys.min()/Hm:.0f}%,{100*ys.max()/Hm:.0f}%] (0,0=图左上)")
print()
print("期望: 左下块 x≈5-13% y≈86-95%(从顶算) | 右上块 x≈87-95% y≈7-16% | 底边线 y≈99%")
