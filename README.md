# Robotics-Control-01-PID-Control
Udacity Self-Driving Car Engineer Nanodegree: PID Control

## P Controller

```python
cte = robot.y
steer = -tau * cte
robot.move(steer, speed)
```

## PD Controller

```python
cte = robot.y
diff_cte = cte - prev_cte
prev_cte = cte
steer = -tau_p * cte - tau_d * diff_cte
robot.move(steer, speed)
```

## PID Controller

```python
cte = robot.y
diff_cte = cte - prev_cte
prev_cte = cte
int_cte += cte
steer = -tau_p * cte - tau_d * diff_cte - tau_i * int_cte
robot.move(steer, speed)
```

## Parameter Optimization

```python
def twiddle(tol=0.2): 
    p = [0, 0, 0]
    dp = [1, 1, 1]
    robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, p)

    it = 0
    while sum(dp) > tol:
        print("Iteration {}, best error = {}".format(it, best_err))
        for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                robot = make_robot()
                x_trajectory, y_trajectory, err = run(robot, p)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
        it += 1
    return p
```
