# 剛体

## 基本

### 力
- 力 (Forces)

    $F = ma$

- トルク (Torque)
    
    $\vec{\tau} = I \cdot \vec{\alpha} = \vec{r} \times \vec{F}$

### 運動量

- 運動量 (Momemtum)

    $P = mv$

- 角運動量 (Angular Momentum)

    $\vec{L} = I \cdot \vec{\omega} = \vec{r} \times \vec{P}$

### 力積

- 力積 (Impulse) ... 運動量の変化

    $J = F dt = m dv$
   
    より速度変化は $dv = \frac{J}{ms}$ 

    $\vec{J} = \vec{\tau} \cdot dt = I \cdot d\omega$
    
    より角速度変化は $d\omega = I^{-1} \cdot \vec{J}$

- 力積と角力積

    $\vec{J}_{angular} = \vec{r} \times \vec{J}_{linear}$

### 運動エネルギー

- 運動エネルギー (Kinetic Energy)

    $T = \frac{1}{2} m v^2$


## 回転している物体
- 回転物体は内部トルクを持つ

    $\vec{\tau} = \vec{\omega} \times I \cdot \vec{\omega}$

    回転物体の角加速度は

    $\vec{\tau} = I \cdot \vec{\alpha}$ 
    
    より

    $\vec{\alpha} = I^{-1} \cdot (\vec{\omega} \times I \cdot \vec{\omega})$

## 衝突力積
- 法線方向

    $\vec{J} = \frac{(1 + \epsilon) v_n}{m^{-1}_1+m^{-1}_2 + (I^{-1}_1 (\vec{r}_1 \times \vec{n}) \times \vec{r}_1 + I^{-1}_2 (\vec{r}_2 \times \vec{n}) \times \vec{r}_2) \cdot \vec{n}}$

- 接線方向

    $\vec{J} = \frac{\mu v_t}{m^{-1}_1+m^{-1}_2 + (I^{-1}_1 (\vec{r}_1 \times \vec{t}) \times \vec{r}_1 + I^{-1}_2 (\vec{r}_2 \times \vec{t}) \times \vec{r}_2) \cdot \vec{t}}$