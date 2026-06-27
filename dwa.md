1. DWA算法原理
1.1 简介
动态窗口算法(Dynamic Window Approaches, DWA) 是基于预测控制理论的一种次优方法，因其在未知环境下能够安全、有效的避开障碍物, 同时具有计算量小, 反应迅速、可操作性强等特点。

DWA算法属于局部路径规划算法。

DWA算法的核心思想是根据移动机器人当前的位置状态和速度状态在速度空间 ( v , ω ) (v, \omega)(v,ω) 中确定一个满足移动机器人硬件约束的采样速度空间，然后计算移动机器人在这些速度情况下移动一定时间内的轨迹， 并通过评价函数对该轨迹进行评价，最后选出评价最优的轨迹所对应的速度来作为移动机器人运动速度， 如此循环直至移动机器人到达目标点。

对于无人驾驶汽车而言，情况类似，将车辆的位置变化转化为线速度和角速度控制，避障问题转变成空间中的运动约束问题，这样可以通过运动约束条件选择局部最优的路径。

1.2 算法原理
DWA算法的思路是：先通过机器人数学模型采集机器人速度样本，并预测模拟出在样本速度下一段时间内生成的运动轨迹， 并对这些运动轨迹进行标准评价， 选择出一组最优轨迹，机器人按照最优轨迹运动。机器人的运动姿态和方向是由机器人当前的线速度及角速度 (转向速度) 共同决定的。

DWA算法主要包括3个步骤：速度采样、轨迹预测（推算）、轨迹评价。

1. 速度采样
由于移动机器人硬件、结构和环境等限制条件，移动机器人的速度采样空间 V s \mathbf{V}_{\mathrm{s}}V 
s
​
  中 ( v , ω ) (v, \omega)(v,ω) 有一定的范围限制。该限制主要分为三大类：

速度边界限制

根据移动机器人的硬件条件和环境限制, 移动机器人的速度存在的边界限制, 此时可采样的速度空间 V m \mathbf{V}_mV 
m
​
  为
V m = { ( v , ω ) ∣ v ∈ [ v min  , v max  ] , ω ∈ [ ω min  , ω max  ] } (1) \tag{1} \mathbf{V}_m=\left\{(v, \omega) \mid v \in\left[v_{\text {min }}, v_{\text {max }}\right], \omega \in\left[\omega_{\text {min }}, \omega_{\text {max }}\right]\right\}
V 
m
​
 ={(v,ω)∣v∈[v 
min 
​
 ,v 
max 
​
 ],ω∈[ω 
min 
​
 ,ω 
max 
​
 ]}(1)

式中 v min  、 v max  v_{\text {min }} 、 v_{\text {max }}v 
min 
​
 、v 
max 
​
  分别为移动机器人最小线速度和最大线速度, ω min  、 ω max ⁡ \omega_{\text {min }} 、 \omega_{\max }ω 
min 
​
 、ω 
max
​
  分别 为移动机器人最小角速度和最大角速度。

加速度限制

由于移动机器人的驱动电机的限制， 故移动机器人的线加速度和角加速度均存在边界限制，假设最大加速度和减速度大小一样，故考虑加速度时可采样的速度空间 V d \mathbf{V}_dV 
d
​
  为
V d = { ( v , ω ) ∣ v ∈ [ v c − a v max ⁡ ⋅ Δ t , v c + a v max ⁡ ∙ Δ t ] ω ∈ [ ω c − a w max ⁡ ∙ Δ t , ω c + a w max ⁡ ∙ Δ t ] } (2) \tag{2} \mathbf{V}_d=\left\{(v, \omega) \mid [Math Processing Error]
\right\}
V 
d
​
 ={(v,ω)∣ 
v∈[v 
c
​
 −a 
vmax
​
 ⋅Δt,v 
c
​
 +a 
vmax
​
 ∙Δt]
ω∈[ω 
c
​
 −a 
wmax
​
 ∙Δt,ω 
c
​
 +a 
wmax
​
 ∙Δt]
​
 }(2)

式中 v c 、 ω c v_c 、 \omega_cv 
c
​
 、ω 
c
​
  分别为移动机器人当前时刻的线速度和角速度，a v max ⁡ 、 a w  max  a_{v \max } 、 a_{w \text { max }}a 
vmax
​
 、a 
w max 
​
  分别为 移动机器人最大线加速度和最大角加速度。

环境障碍物限制

局部规划还需要有动态实时的避障功能。考虑移动机器人的周围的障碍物因素，某一时刻移动机器人不与周围障碍物发生碰撞的可约束条件为
V a = { ( v , ω ) ∣ v ∈ [ v min ⁡ , 2 ⋅ d i s t ( v , ω ) ⋅ a v max ⁡ ] ω ∈ [ ω min ⁡ , 2 ⋅ d i s t ( v , ω ) ⋅ a ω max ⁡ ] } (3) \tag{3} \mathbf{V}_a=\left\{(v, \omega) \mid [Math Processing Error]
\right\}
V 
a
​
 = 
⎩
⎨
⎧
​
 (v,ω)∣ 
v∈[v 
min
​
 , 
2⋅dist(v,ω)⋅a 
vmax
​
 
​
 ]
ω∈[ω 
min
​
 , 
2⋅dist(v,ω)⋅a 
ωmax
​
 
​
 ]
​
  
⎭
⎬
⎫
​
 (3)

式中dist ⁡ ( v , ω ) \operatorname{dist}(v, \omega)dist(v,ω)表示当前速度下对应模拟轨迹与障碍物之间的最近距离。 在无障碍物的情况下 dist ⁡ ( v , ω ) \operatorname{dist}(v, \omega)dist(v,ω) 会是一个很大的常数值。当机器人运行采样速度在公式 (3) 范围时, 能够以最大减速度的约束实现安全减速直至避开障碍物。

注意: 这个限制条件在采样初期是得不到的，需要我们先使用V m ∩ V d \mathbf{V}_m \cap \mathbf{V}_dV 
m
​
 ∩V 
d
​
 的速度组合采样模拟出轨迹后, 计算当前速度下对应模拟轨迹与障碍物之间的最近距离, 然后看当前采样的这对速度能否在碰到障碍物之前停下来， 如果能够停下来, 那这对速度就是可接收的。如果不能停下来, 这对速度就得抛弃掉。

我在代码中并没有采用这种做法，而是直接计算机器人当前位置（并不是模拟轨迹）与障碍物的最近距离来得到V a \mathbf{V}_aV 
a
​
 ，算是一种比较投机的做法。

结合上述三类速度限制， 最终的移动机器人速度采样空间是三个速度空间的交集，即
V s = V m ∩ V d ∩ V a (4) \tag{4} \mathbf{V}_s=\mathbf{V}_m \cap \mathbf{V}_d \cap \mathbf{V}_a
V 
s
​
 =V 
m
​
 ∩V 
d
​
 ∩V 
a
​
 (4)

2. 轨迹预测（轨迹推算）
在确定速度采样空间 V s \mathbf{V}_sV 
s
​
  后，DWA算法以一定的采样间距（分辨率）在该空间均匀采样。

在速度空间中, 分别对线速度和角速度设置分辨率, 分别用 E w 、 E v E_w 、 E_vE 
w
​
 、E 
v
​
  表示采样分辨率，那么采样速度组的个数就可以确定下来, 如下式所示。
n = [ ( v h i g h − v l o w ) / E v ] ⋅ [ ( w h i g h − w l o w ) / E w ] n=\left[\left(v_{high}-v_{low }\right) / E_v\right]\cdot\left[\left(w_{high }-w_{low }\right) / E_w\right]
n=[(v 
high
​
 −v 
low
​
 )/E 
v
​
 ]⋅[(w 
high
​
 −w 
low
​
 )/E 
w
​
 ]

式中的v h i g h , v l o w , w h i g h , w l o w v_{high},v_{low },w_{high },w_{low }v 
high
​
 ,v 
low
​
 ,w 
high
​
 ,w 
low
​
 是速度空间的上下限。

上式说明线速度每间隔一个 E v E_vE 
v
​
  大小取一个值, 角速度每间隔一个 E w E_wE 
w
​
  大小取一个值，由此组成了一系列的速度组。

当采样了一组 ( v , ω ) (v, \omega)(v,ω) 后, 通过移动机器人（无人车辆）的运动学模型进行轨迹预测（即位置更新）。

因此，在轨迹预测阶段，只需要知道机器人（无人车辆）的运动学模型即可，然后在预测时间内保存预测轨迹便于后面处理。有关无人车的运动学模型可参考这篇博客。

这边不妨使用差分驱动移动机器人的运动学模型，那么轨迹预测就是计算下式：
x k = x k − 1 + v ⋅ cos ⁡ ( θ k − 1 ) Δ t y k = y k − 1 + v ⋅ sin ⁡ ( θ k − 1 ) Δ t θ k = θ k − 1 + ω Δ t [Math Processing Error]
x 
k
​
 
y 
k
​
 
θ 
k
​
 
​
  
=x 
k−1
​
 +v⋅cos(θ 
k−1
​
 )Δt
=y 
k−1
​
 +v⋅sin(θ 
k−1
​
 )Δt
=θ 
k−1
​
 +ωΔt
​
 

式中，( x , y , θ ) (x,y,\theta)(x,y,θ)代表机器人的位姿，k kk代表采样时刻，Δ t \Delta tΔt表示采样间隔。

上面轨迹推算这块是我比较肤浅的理解，因为按照原论文是需要假设相邻时间段内机器人的轨迹是圆弧，然后再进行轨迹推算，感兴趣的朋友可以直接查看原论文。

3. 轨迹评价
确定了机器人约束速度范围后，有一些速度模拟的轨迹是可行的， 但是还有不达标的轨迹， 这需要对采样得到的多组轨迹进行评价择优。通过标准评价轨迹， 比较评分来选出最优轨迹， 然后选取最优轨迹对应的速度作为驱动速度。对每条轨迹进行评估 的评价函数如公式 ( 5 ) (5)(5) 所示。
G ( v , ω ) = σ ( α ⋅ heading ⁡ ( v , ω ) ) + σ ( β ⋅ dist ⁡ ( v , ω ) ) + σ ( γ ⋅ velocity ⁡ ( v , ω ) ) (5) \tag{5} G(v, \omega)=\sigma(\alpha \cdot \operatorname{heading}(v, \omega))+\sigma(\beta \cdot \operatorname{dist}(v, \omega))+\sigma(\gamma \cdot \operatorname{velocity}(v, \omega))
G(v,ω)=σ(α⋅heading(v,ω))+σ(β⋅dist(v,ω))+σ(γ⋅velocity(v,ω))(5)

heading ( v , ω ) (v, \omega)(v,ω) 是方位角评价函数， 用作评估在当前采样速度下产生的轨迹终点位置方向与目标点连线的夹角的误差Δ θ \Delta\thetaΔθ；由于我们想要用评价函数越大表示越优，所以用π − Δ θ \pi-\Delta\thetaπ−Δθ来参与评价，即h e a d i n g ( v , ω ) = π − Δ θ heading (v, \omega)= \pi-\Delta\thetaheading(v,ω)=π−Δθ。


dist ⁡ ( v , ω ) \operatorname{dist}(v, \omega)dist(v,ω) 是距离评价函数， 表示当前速度下对应模拟轨迹与障碍物之间的最近距离；如果没有障碍物或者最近距离大于设定的阈值，那么就将其值设为一个较大的常数值。

velocity ( v , ω ) (v, \omega)(v,ω) 是速度评价函数， 表示当前的速度大小，可以直接用当前线速度的大小来表示。它越大，表示规划轨迹上的速度越快，评价得分越高。

以上三种评价函数只是给了个大体的意思，并不绝对，例如有的人是把评价函数作为代价，代价越小，轨迹越优。可根据自己的想法进行评价函数的设置，但无论怎么变，这三种评价函数都是需要的。

α 、 β \alpha 、 \betaα、β 和 γ \gammaγ 均为评价函数的系数。由于局部路径规划的过程需要多传感器的采集, 采集信息无法做到连续, 这样也会使得评价后差别较大, 所以可以进行归一化处理（平滑处理）， 其中 σ \sigmaσ 表示 归一化。

归一化处理过程如下式 (6) 所示:
σ ⋅  heading  ( v , ω ) = normalize-heading ( i ) = heading ⁡ ( i ) ∑ i = 1 n heading ⁡ ( i ) σ ⋅ dist ⁡ ( v , ω ) =  normalize-dist  ( i ) = dist ⁡ ( i ) ∑ i = 1 n dist ⁡ ( i ) σ ⋅ velocity ⁡ ( v , ω ) =  normalize-velocity  ( i ) =  velocity  ( i ) ∑ i = 1 n  velocity  ( i ) (6) \tag{6} [Math Processing Error]
​
  
σ⋅ heading (v,ω)=normalize-heading(i)= 
∑ 
i=1
n
​
 heading(i)
heading(i)
​
 
σ⋅dist(v,ω)= normalize-dist (i)= 
∑ 
i=1
n
​
 dist(i)
dist(i)
​
 
σ⋅velocity(v,ω)= normalize-velocity (i)= 
∑ 
i=1
n
​
  velocity (i)
 velocity (i)
​
 
​
 (6)

其中，i ii 代表第i ii条模拟轨迹，n nn 为约束条件下的全部采样轨迹总数。由上述公式 (5) 和公式 (6) 可以得出一条满足避开障碍物并朝着目标点快速行进的路径， 使得机器人完成局部路径最优规划。
————————————————
版权声明：本文为CSDN博主「CHH3213」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/weixin_42301220/article/details/127769819