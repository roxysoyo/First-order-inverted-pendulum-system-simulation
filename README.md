# 一阶倒立摆系统仿真

[TOC]

# 1. 仿真软件

本次仿真所采用的求解软件是MATLAB。



## 2. 倒立摆动态系统建模

### 2.1 倒立摆系统的简化

我们可以简化倒立摆系统为如图1所示的形式，简单来说就是通过调控滑块运动状态，使杆直立不倒。

<center>
    <img src="https://gitee.com/roxysoyo/picgo-img/raw/master/img/202312211818594.png" width=50%>
    <br>
    图1 倒立摆模型示意图
</center>


下表是倒立摆系统的各个参数：

| 符号 |          物理含义          |    参数值     |
| :--: | :------------------------: | :-----------: |
|  M   |          滑块质量          |   0.2275kg    |
|  m   |          摆杆质量          |   0.0923kg    |
|  l   | 摆杆转动轴心到杆质心的长度 |    0.185m     |
|  J   |        摆杆转动惯量        | 0.00412kg*m^2 |



### 2.2 倒立摆系统运动学方程推导

接下来我们来对倒立摆系统的运动学方程进行推导。如图2所示，摆杆长度为$2l$，倾角为$\varphi$。我们设滑块质心位置为$(X,0)$，摆杆质心位置为$(X_m,Y_m)$。

<center>
    <img src="https://gitee.com/roxysoyo/picgo-img/raw/master/img/202312211824020.png" width=50%>
    <br>
    图2 坐标示意图
</center>



我们可以列出如下表达式：
$$
\begin{align}
X_m&=X-l\cdot \sin \varphi \tag{1}\\
Y_m&=l\cdot \cos \varphi \tag{2}\\
\end{align}
$$
对$(1),(2)$式分别求对时间$t$的一阶导数，可得到如下表达式：
$$
\begin{align}
\dot{X_m}&=\dot{X}-l \cdot \dot{\varphi} \cdot \cos \varphi \tag{3}\\
\dot{Y_m}&=-l \cdot \dot{\varphi} \cdot\sin \varphi \tag{4}\\
\end{align}
$$
基础表达式已经写好，下面我们可以使用拉格朗日力学方程来建立倒立摆系统方程。拉格朗日力学方程如下：


$$
\begin{align}
&\mathcal{L}=T-V\\
&\frac{d}{dt}(\frac{\partial \mathcal{L}}{\partial \dot{q_{i}}})-\frac{\partial \mathcal{L}}{\partial q_i}=Q_i\\
\end{align}
$$
其中$T,V$分别是这个力学体系的动能和势能，$Q_i$为$q_i$所对应的非保守的广义力。为了简便计算，我们做出如下规定：滑块所在平面的势能$V_M=0$，则摆杆势能为$V_m=mgl\cos \varphi$，总势能为$V=V_M+V_m=mgl\cos \varphi$。接下来关于动能$T$的推导会相对复杂一些，因为摆杆的动能包括了平动和转动。
$$
\begin{align}
T&=\frac{1}{2}M\dot{X}^2+\frac{1}{2}m(X_m^2+Y_m^2)+\frac{1}{2}J\omega^2\\
&=\frac{1}{2}M\dot{X}^2+\frac{1}{2}m(\dot{X}^2-2l\dot{X}\dot{\varphi}\cos \varphi+l^2\dot{\varphi}^2\cos^2 \varphi+l^2\dot{\varphi}^2\sin^2 \varphi)+\frac{1}{2}J\omega^2\\
&=\frac{1}{2}M\dot{X}^2+\frac{1}{2}m(\dot{X}^2-2l\dot{X}\dot{\varphi}\cos \varphi+l^2\dot{\varphi}^2)+\frac{1}{2}J\omega^2\\
&=\frac{1}{2}(M+m)\dot{X}^2+\frac{1}{2}{m}l^2\dot{\varphi}^2-ml\dot{X}\dot{\varphi}\cos \varphi+\frac{1}{2}J\omega^2\\
\end{align}
$$

$$
\begin{align}
\mathcal{L}=T-V
=\frac{1}{2}(M+m)\dot{X}^2+\frac{1}{2}{m}l^2\dot{\varphi}^2-ml\dot{X}\dot{\varphi}\cos \varphi+\frac{1}{2}J\omega^2-mgl\cos \varphi \tag{5}\\
\end{align}
$$

下面需要将式$(5)$代入到拉格朗日力学方程中，对于本系统来说，拉格朗日力学方程如下：
$$
\begin{align}
\dfrac{d}{dt}(\frac{\partial \mathcal{L}}{\partial \dot{X}})-\frac{\partial \mathcal{L}}{\partial X}&=F\\
\dfrac{d}{dt}(\frac{\partial \mathcal{L}}{\partial \dot{\varphi}})-\frac{\partial \mathcal{L}}{\partial \varphi}&=0 \tag{6}\\
\end{align}
$$
式子$(5)$：$\mathcal{L}=T-V
=\frac{1}{2}(M+m)\dot{X}^2+\frac{1}{2}{m}l^2\dot{\varphi}^2-ml\dot{X}\dot{\varphi}\cos \varphi+\frac{1}{2}J\omega^2-mgl\cos \varphi \tag{5}\\$，代入后的结果如下：
$$
\begin{align}
X:\\
&\frac{\partial \mathcal{L}}{\partial \dot{X}}=(M+m)\dot{X}-ml\dot{\varphi}\cos \varphi\\
&\frac{\partial \mathcal{L}}{\partial X}=0\\
&\dfrac{d}{dt}(\frac{\partial \mathcal{L}}{\partial \dot{X}})=(M+m)\ddot{X}-ml(\ddot{\varphi}\cos\varphi-\dot{\varphi}^2\sin \varphi)\\
\therefore \quad &\dfrac{d}{dt}(\frac{\partial \mathcal{L}}{\partial \dot{X}})-\frac{\partial \mathcal{L}}{\partial X}\\
&=(M+m)\ddot{X}-ml(\ddot{\varphi}\cos\varphi-\dot{\varphi}^2\sin \varphi)=F \tag{7}\\

\varphi:\\
&\frac{\partial \mathcal{L}}{\partial \dot{\varphi}}=ml^2\dot{\varphi}-ml\dot{X}\cos \varphi+J\dot{\varphi}\\
&\frac{\partial \mathcal{L}}{\partial \varphi}=ml\dot{X}\dot{\varphi}\sin \varphi+mgl\sin \varphi\\

&\dfrac{d}{dt}(\frac{\partial \mathcal{L}}{\partial \dot{\varphi}})=ml^2\ddot{\varphi}-ml(\ddot{X}\cos\varphi-\dot{X}\dot{\varphi}\sin \varphi)+J\ddot{\varphi}\\



\therefore \quad &\dfrac{d}{dt}(\frac{\partial \mathcal{L}}{\partial \dot{\varphi}})-\frac{\partial \mathcal{L}}{\partial \varphi}\\

&=ml^2\ddot{\varphi}-ml(\ddot{X}\cos\varphi-\dot{X}\dot{\varphi}\sin \varphi)+J\ddot{\varphi}-ml\dot{X}\dot{\varphi}\sin \varphi-mgl\sin \varphi\\

&=ml^2\ddot{\varphi}-ml\ddot{X}\cos\varphi+J\ddot{\varphi}-mgl\sin \varphi\\

&=l\ddot{\varphi}-\ddot{X}\cos\varphi+\frac{J}{ml}\ddot{\varphi}-g\sin \varphi=0 \tag{8}\\
\end{align}
$$

至此，倒立摆系统的动力学方程推导完毕。



### 2.3 线性化处理

 上文得到的式$(7),(8)$，可以看出，倒立摆系统的为非线性微分方程，为了后面数学分析的简便性和应用LQR控制器进行反馈控制，我们需要对式$(7),(8)$进行线性化处理。

当$\varphi \rightarrow 0$时，$\sin \varphi \rightarrow \varphi,\cos \varphi \rightarrow 1$，并且$\dot{\varphi}^2$是$\varphi$的高阶无穷小，因此$\dot{\varphi}^2\sin \varphi \rightarrow 0$。

线性化处理后，得到如下微分方程组：
$$
\begin{cases}
(M+m)\ddot{X}-ml\ddot{\varphi}=F\\
l\ddot{\varphi}-\ddot{X}+\dfrac{J}{ml}\ddot{\varphi}-g\varphi=0 \tag{9}\\
\end{cases}
$$
接下来我们会建立倒立摆系统的状态空间方程。




## 3. 倒立摆状态空间反馈控制

### 3.1 状态空间方程一般形式

系统状态空间方程一般形式如下所示：
$$
\begin{cases}
\dot{Z}=AZ+BU\\
Y=CZ+DU\\
\end{cases}\tag{10}
$$
其中，向量$Z$代表系统状态变量，向量$U$代表系统输入，向量$Y$代表系统输出，矩阵$A,B,C,D$为系统状态矩阵。

另外，状态空间方程与经典控制理论中的传递函数$G(s)$关系如下：
$$
G(s)=C(SI-A)^{-1}B+D \tag{11}
$$

### 3.2 倒立摆系统状态空间方程

结合上面所说的状态空间方程的形式，我们需要将倒立摆系统微分方程化为同样的形式。在倒立摆系统中，我们主要关注这四个状态量：滑块的位移$X$与速度$\dot{X}/v$、摆杆的倾角$\varphi$与角速度$\dot{\varphi}/\omega$。所以，我们做出如下规定：
$$
Z=
\begin{bmatrix}
z_1\\z_2\\z_3\\z_4
\end{bmatrix}\\
$$

$$
\begin{align}
z_1&=X\\
z_2&=\dot{X}\\
z_3&=\varphi\\
z_4&=\dot{\varphi}\\
\end{align}
$$

之后我们可以将倒立摆动态系统微分方程化为如下形式：
$$
\begin{align}
(M+m)\dot{z_2}-ml\dot{z_4}&=F\\
l\dot{z_4}-\dot{z_2}+\dfrac{J}{ml}\dot{z_4}-gz_3&=0 \tag{12}\\
\end{align}
$$

通常，在状态空间方程中，习惯性将系统输入记作$U$，因此这里我们令$U=F$。为了表达式的简便性，我们记$a=\dfrac{J}{ml}$。随后将微分方程化为矩阵形式，即为倒立摆开环状态空间方程了：
$$
\begin{bmatrix}
\dot{z_1}\\
\dot{z_2}\\
\dot{z_3}\\
\dot{z_4}\\
\end{bmatrix}
=
\begin{bmatrix}
0 & 1 & 0 & 0\\
0 & 0 & \dfrac{mgl}{Ml+Ma+ma} & 0\\
0 & 0 & 0 & 1\\
0 & 0 & \dfrac{(M+m)g}{Ml+Ma+ma} & 0\\
\end{bmatrix}
\cdot
\begin{bmatrix}
z_1\\
z_2\\
z_3\\
z_4\\
\end{bmatrix}
+
\begin{bmatrix}
0\\
\dfrac{l+a}{Ml+Ma+ma}\\
0\\
\dfrac{1}{Ml+Ma+ma}\\
\end{bmatrix}
\cdot U \tag{13}
$$

### 3.3 加入状态反馈控制

根据式$(10)$中$\dot{Z}=AZ+BU$，我们令$U=-KZ$，得到：
$$
\dot{Z}=(A-BK)Z+BU \tag{14}
$$
其中，$K=[k_1,k_2,k_3,k_4]$。此外，我们一般把矩阵$(A-BK)$称作闭环控制状态空间矩阵$A_{cl}$。系统闭环状态反馈控制示意图如图3所示。

<center>
    <img src="https://gitee.com/roxysoyo/picgo-img/raw/master/img/202312221418491.png" width=75%>
    <br>
    图3 系统闭环状态反馈控制
</center>

之后，我们把$U=-KZ$带入到式$(11)$中去：
$$
\begin{bmatrix}
\dot{z_1}\\
\dot{z_2}\\
\dot{z_3}\\
\dot{z_4}\\
\end{bmatrix}
=
\begin{bmatrix}
0 & 1 & 0 & 0\\
0 & 0 & \dfrac{mgl}{Ml+Ma+ma} & 0\\
0 & 0 & 0 & 1\\
0 & 0 & \dfrac{(M+m)g}{Ml+Ma+ma} & 0\\
\end{bmatrix}
\cdot
\begin{bmatrix}
z_1\\
z_2\\
z_3\\
z_4\\
\end{bmatrix}
+
\begin{bmatrix}
0\\
\dfrac{l+a}{Ml+Ma+ma}\\
0\\
\dfrac{1}{Ml+Ma+ma}\\
\end{bmatrix}
\cdot
\begin{bmatrix}
-k_1 & -k_2 & -k_3 & -k_4
\end{bmatrix}
\cdot
\begin{bmatrix}
z_1\\
z_2\\
z_3\\
z_4\\
\end{bmatrix}
\tag{15}
$$



## 4. LQR控制器实现反馈控制

### 4.1 LQR控制器

在现代控制理论中，在我们得到矩阵$A_{cl}$后，求其特征值，就能够知道控制系统的表现能力，如系统是否能稳定、是否能快速收敛到稳定值等等。因此，在设计控制器时，重点就是如何配置矩阵$A_{cl}$的特征值，如同在经典控制理论中配置传递函数的极点。这时我们引入了LQR控制器。

LQR一般形式表示如下：
$$
\begin{align}
J=\int_0^\infin(x^TQx+u^TRu)dt
\end{align}
$$
其中， $x$为被控对象的状态变量； $u$为被控对象的输入量；矩阵 $Q$和 $R$分别代表状态变量 $x$ 及输入量 $u$的权重，$Q,R$均为对角矩阵，对角线中的值代表权重系数。关于LQR控制器的详细推导请参考这篇[文章](https://zhuanlan.zhihu.com/p/147473204)。

### 4.2 Matlab求解

下面我们使用Matlab计算出矩阵$K$，并求解倒立摆系统的微分方程（数值解）。

```matlab
%% lqrcontrol.m
clear all;
%% 系统参数
M = 0.2275;
m = 0.0923;
g = 9.8;
L = 0.185;
J = 0.00412;

%% 状态空间矩阵
a=J/(m*L);
fen_mu=M*L+M*a+m*a;

%% 状态空间矩阵
A=[
    0 1 0 0;
    0 0 m*g*L/fen_mu 0;
    0 0 0 1;
    0 0 (m+M)*g/fen_mu 0
    ];
B=[0;(L+a)/fen_mu;0;1/fen_mu];
C=eye(4);
D=0;

%% 建立系统
cart=ss(A,B,C,D);

%% LQR
Q = diag([10 1 10 1]);% x,dot(x),phi,dot(phi)
R = 0.001;
% 计算出k
K =lqr(cart,Q,R);
% 计算Acl的特征值，观察系统稳定性
eig(A-B*K)

%% 求解微分方程
tspan=0:0.05:10;
y_0=[0;0;pi/12;0];
[t,y] = ode45(@(t,y)solve_cart(t,y,m,M,L,g,J,K),tspan, y_0);

figure;
grid on;
hold on;
plot(tspan,y(:,1));
plot(tspan,y(:,2));
plot(tspan,y(:,3));
plot(tspan,y(:,4));
legend({'x', '$$\dot{x}$$', '$$\varphi$$', '$$\dot{\varphi}$$'}, 'Interpreter', 'latex');
hold off
```

运行$lqrcontrol.m$文件后，就能够获得倒立摆系统的四个状态量随时间变化趋势。

下面是摆杆初始倾角$\varphi_0=15\degree$的状态量变化图：

<center>
    <img src="https://gitee.com/roxysoyo/picgo-img/raw/master/img/202312221624309.png" width=100%>
    <br>
    图4 初始倾角为15°时，倒立摆系统的状态量变化图
</center>

从上图能够清晰地看到，摆杆在滑块的运动控制下3s内保持直立平衡，响应速度较快，系统输入量也较为合适，避免了过高要求的输入。

下面我们通过Matlab中的Simulink进行倒立摆系统的仿真。



## 5. Simulink仿真建模与动画效果

### 5.1 仿真环境搭建

我们通过Matlab中的Simulink模块来搭建仿真环境，并借助Simulink中的Simscape搭建倒立摆系统的物理仿真。

<center>
    <img src="https://gitee.com/roxysoyo/picgo-img/raw/master/img/202312221954202.png" width=100%>
    <br>
    图5 Simulink仿真环境总览
</center>

从上图我们可以看到，整个仿真环境分为2个Subsystem，左边是Controller，右边是倒立摆物理系统Cart System仿真，两边组成了一个闭环控制系统。我们首先来看Cart System：

<center>
    <img src="https://gitee.com/roxysoyo/picgo-img/raw/master/img/202312222002451.png" width=50%>
    <br>
    图6 Cart System建模总览
</center>

Cart System环境配置步骤如下：

1. 设置Simulink求解器、世界坐标系和机械配置（模拟重力加速度）；
2. 添加所需的物理实体，包括导轨地面、滑块、摆杆、转轴以及旋转关节。对于滑块来说，它是系统的主动元件，配置一个三轴自由度的属性（实际滑块是X轴单轴运动），随后对各个实体设置好合适的大小以及坐标系。上述滑块由Solidworks软件建模得到并导入Simscape中；
3. 在滑块与导轨地面间添加实体碰撞效果Spatial Contact；
4. 由于Simscape中的状态量和Simulink中的状态量数据类型并不一致，所以需要设置好相应的数据转换器；
5. 将需要的四个状态量输出，并用scope模块观察。

 下面我们来看Controller：

<center>
    <img src="https://gitee.com/roxysoyo/picgo-img/raw/master/img/202312222015155.png" width=50%>
    <br>
    图7 Controller建模总览
</center>

Controller配置步骤如下：

1. 将Cart System的输出（四个状态量）作为控制器的输入；
2. 添加Rate Transition，并设置Controller模块每隔0.001s触发后面的Matlab求解方程；
3. 添加Interpreted MATLAB Function，设置预先写好的Matlab控制算法脚本（具体内容在5.2节）。

<center>
    <img src="https://gitee.com/roxysoyo/picgo-img/raw/master/img/202312222020099.png" width=75%>
    <br>
    图8 Interpreted MATLAB Function具体设置
</center>



### 5.2 仿真环境所需代码

代码编写步骤如下：

1. 编写倒立摆系统初始参数、控制算法函数脚本$startUp.m$。在运行脚本后，仿真模块能从Matlab工作区查找到系统初始参数，以及函数名；
2. 编写LQR控制器脚本$cartLQR.m$，与上文$lqrcontrol.m$文件求解微分方程过程一只，去除掉了绘图部分；
3. 编写反馈控制脚本$LQRControl.m$，也就是状态反馈控制中的$U=-KX$的代码实现，如下所示：

```matlab
%% LQRControl.m
function Fx = LQRControl(states)
global K_LQR
%% desired value
X_des = [0;0;0;0];
%% control law
Fx = K_LQR*(X_des - states);
end
```



我们在Cart System中添加了Scope模块，可以在仿真模型运行后观察倒立摆系统的四个状态量，如图10所示：

<center>
    <img src="https://gitee.com/roxysoyo/picgo-img/raw/master/img/202312222138749.png" width=100%>
    <br>
    图10 仿真环境下的四个状态量
</center>


至此，关于一阶倒立摆系统的建模仿真结束。
