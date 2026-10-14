---
title: "MPPI: Model Predictive Control by Sampling"
date: 2026-08-16
permalink: /posts/2026/08/mppi-sampling-based-mpc/
toc: true
excerpt: "Why gradient-based MPC stalls on non-convex problems, how importance sampling gets us out, and what MPPI looks like on a dynamic lane-merge."
tags:
  - MPPI
  - model predictive control
  - sampling-based control
  - motion planning
---

Classical MPC solves a nonlinear program at every timestep. When the problem is
convex and smooth, that is a good deal. When it is neither, think obstacle
avoidance, contact, or discontinuous costs, the solve time is what eventually
breaks the controller.

This post walks through Model Predictive Path Integral control (MPPI), which
replaces the solver with sampling: roll out a few thousand random control
sequences, score them, and take a weighted average. The only real mathematical
ingredient is importance sampling, so we build that up first.

## MPC and the solve-time problem

Model Predictive Control is an optimal control method that uses a model to roll
out (predict) over a horizon and compute the best control, using an optimizer
such as IPOPT or SQP, applied in a receding-horizon manner.

Define a cost

$$
J(u) = \frac{1}{2}\sum_{k=0}^{N-1}\left(x_k^\top Q x_k + u_k^\top R u_k\right) + \phi(x_N),
$$

where $$Q \succeq 0$$, $$R \succ 0$$, and $$\phi(x_N)$$ is the terminal cost. We then
solve for the optimal control sequence

$$
u^{*} = \underset{u_0,\ldots,u_{N-1}}{\arg\min} \; J(u)
$$

subject to

$$
x_{k+1} = f(x_k, u_k), \qquad k = 0,\ldots,N-1.
$$

Once the optimal sequence $$u^{*} = \{u_0^{*},\ldots,u_{N-1}^{*}\}$$ is obtained,
only the first control $$u_0^{*}$$ is applied. That is the receding-horizon step,
and the optimization is repeated at every timestep.

**So what is the problem?** Nonlinear dynamics and constraints such as obstacle
avoidance make the optimization non-convex. Non-convex problems need more
compute, which means higher solve times and lower control rates — exactly what
you cannot afford on a vehicle moving at speed.

## Sampling-based MPC

Instead of optimizing for a single trajectory, sampling-based methods maintain a
*distribution* over control sequences. At every timestep we roll out a batch of
candidate sequences, score them, and use those scores to update the
distribution.

That immediately raises three questions: what distribution do we use, how do we
sample from it, and how do we turn costs into an updated control sequence?
Several families of methods answer these differently — path-integral control,
information-theoretic derivations based on relative entropy, stochastic search,
and variational inference. This post takes the stochastic-search route to MPPI.

## Detour: importance sampling

Before MPPI, two ideas: Monte Carlo integration and importance sampling.

### Monte Carlo integration

Consider the integral

$$
I = \int_a^b f(x)\,dx.
$$

One way to estimate it is to sample $$x$$ uniformly from $$[a,b]$$. Let
$$x_i \sim U(a,b)$$ for $$i = 1,\ldots,N$$. The expected value of $$f(x)$$ is

$$
\mathbb{E}[f(x)] = \frac{1}{b-a}\int_a^b f(x)\,dx,
$$

so that $$I = (b-a)\,\mathbb{E}[f(x)]$$. By the law of large numbers the
expectation is approximated by the sample mean,

$$
\mathbb{E}[f(x)] \approx \frac{1}{N}\sum_{i=1}^{N} f(x_i),
$$

and hence

$$
I \approx \frac{b-a}{N}\sum_{i=1}^{N} f(x_i).
$$

That is the whole idea behind Monte Carlo integration: instead of evaluating the
function everywhere in the domain, evaluate it at randomly sampled points. The
error decreases as $$O(1/\sqrt{N})$$ — slowly, but independently of dimension,
which is why the idea survives in high-dimensional control problems.

Uniform sampling, however, is wasteful. Take

$$
f(x) = e^{-x^{2}}.
$$

Sample uniformly over a wide interval and most samples land where $$f(x)$$ is
essentially zero, contributing nothing to the estimate. Which raises the
question: **can we sample more intelligently?**

### Importance sampling

Suppose that instead of sampling from a uniform distribution, we want to sample
from some distribution $$q(x)$$. Consider the expectation

$$
\mu = \mathbb{E}_{p}[f(x)] = \int f(x) p(x)\,dx,
$$

where $$p(x)$$ is the distribution under which we actually want the expectation.
Multiply and divide by $$q(x)$$:

$$
\begin{aligned}
\mu &= \int f(x) p(x)\,dx \\
    &= \int f(x)\frac{p(x)}{q(x)} q(x)\,dx,
\end{aligned}
$$

which gives

$$
\mathbb{E}_{p}[f(x)] = \mathbb{E}_{q}\!\left[f(x)\frac{p(x)}{q(x)}\right].
$$

We can therefore sample from $$q(x)$$ instead of $$p(x)$$, as long as we correct
each sample by the ratio $$w(x) = p(x)/q(x)$$. For $$N$$ samples $$x_i \sim q(x)$$,

$$
\hat{\mu} = \frac{1}{N}\sum_{i=1}^{N} f(x_i)\frac{p(x_i)}{q(x_i)},
$$

where $$w_i = p(x_i)/q(x_i)$$ is the importance weight. The recipe is short:
**sample from $$q$$, then weight by $$p/q$$.** Rather than spreading samples
uniformly across the domain, choose a proposal $$q(x)$$ that puts samples where
they matter.

This — sampling from one distribution and correcting with importance weights —
is the key ingredient in MPPI.

## MPPI

### A little history

The first version of MPPI appeared in 2016 in *Aggressive Driving with Model
Predictive Path Integral Control* out of the AutoRally / ACDS group at Georgia
Tech, designed for GPUs and for aggressive driving. The original formulation came
from path-integral optimal control and information-theoretic ideas, and it leaned
on massively parallel trajectory sampling to be GPU-implementable. Below is the
stochastic-search view, which gets to the same update with less machinery.

### Derivation

Define the optimal distribution over control sequences as a Boltzmann
distribution,

$$
p^{*}(u) \propto \exp\!\left(-\frac{J(u)}{\lambda}\right),
$$

where $$J(u)$$ is the cost of a control trajectory and $$\lambda$$ is a temperature
parameter. Small $$\lambda$$ concentrates mass on the best trajectories; large
$$\lambda$$ spreads it out.

Ideally we would sample directly from $$p^{*}(u)$$, but that is hard: evaluating it
requires rolling out the dynamics and computing the trajectory cost. So we use
importance sampling with a Gaussian proposal centred on the current mean control
sequence,

$$
q(u) = \mathcal{N}(\mu, \Sigma),
$$

or equivalently, we perturb the current control sequence,

$$
u_i = \mu + \epsilon_i, \qquad \epsilon_i \sim \mathcal{N}(0,\Sigma).
$$

From importance sampling,

$$
\mathbb{E}_{p^{*}}[u] = \mathbb{E}_{q}\!\left[u\,\frac{p^{*}(u)}{q(u)}\right],
$$

so with $$N$$ samples,

$$
u^{*} \approx \frac{\sum_{i=1}^{N} u_i w_i}{\sum_{i=1}^{N} w_i},
\qquad
w_i = \frac{p^{*}(u_i)}{q(u_i)}.
$$

Using the Boltzmann form $$p^{*}(u_i) \propto \exp(-J_i/\lambda)$$ and the Gaussian
proposal $$q(u_i) \propto \exp\!\left(-\tfrac{1}{2}\epsilon_i^\top \Sigma^{-1}\epsilon_i\right)$$,

$$
\begin{aligned}
w_i &\propto \exp\!\left(-\frac{J_i}{\lambda}\right)
       \exp\!\left(\frac{1}{2}\epsilon_i^\top \Sigma^{-1}\epsilon_i\right) \\
    &\propto \exp\!\left(-\frac{1}{\lambda}
       \left[J_i - \frac{\lambda}{2}\epsilon_i^\top \Sigma^{-1}\epsilon_i\right]\right).
\end{aligned}
$$

Defining the modified cost

$$
\tilde{J}_i = J_i - \frac{\lambda}{2}\epsilon_i^\top \Sigma^{-1}\epsilon_i,
$$

the weights become

$$
w_i \propto \exp\!\left(-\frac{\tilde{J}_i}{\lambda}\right),
$$

and after normalization,

$$
w_i = \frac{\exp(-\tilde{J}_i/\lambda)}
           {\sum_{j=1}^{N}\exp(-\tilde{J}_j/\lambda)}.
$$

The optimal control is then the weighted average of the sampled sequences,
$$u^{*} \approx \sum_{i=1}^{N} w_i u_i$$. Since $$u_i = \mu + \epsilon_i$$,

$$
\begin{aligned}
u^{*} &\approx \sum_{i=1}^{N} w_i(\mu + \epsilon_i) \\
      &= \mu\sum_{i=1}^{N} w_i + \sum_{i=1}^{N} w_i \epsilon_i,
\end{aligned}
$$

and because the weights are normalized, $$\sum_i w_i = 1$$, so

$$
u^{*} \approx \mu + \sum_{i=1}^{N} w_i \epsilon_i.
$$

Instead of averaging the sampled control sequences directly, we update the mean
using the weighted perturbations:

$$
\mu \;\leftarrow\; \mu + \sum_{i=1}^{N} w_i \epsilon_i.
$$

That is the MPPI control update. As before, only the first control input of the
updated sequence is applied; the sequence is then shifted forward and the process
repeats at the next timestep.

> **A note on the weight.** The information-theoretic derivation of MPPI
> (Williams et al.) defines the optimal distribution relative to a *base*
> uncontrolled distribution, and the resulting modified cost picks up a
> cross term $$\lambda\,\mu^\top \Sigma^{-1}\epsilon_i$$ rather than the
> $$-\tfrac{\lambda}{2}\epsilon_i^\top\Sigma^{-1}\epsilon_i$$ above. Many
> implementations simply drop the control-dependent term altogether and weight on
> the raw state cost, which works fine in practice when $$\lambda$$ is tuned. Worth
> knowing which convention a given codebase follows before comparing gains.

### Pseudocode

```text
given: nominal control sequence U = [u_0, ..., u_{N-1}]

loop at every control step:
    x_0 <- get_state()

    for i = 1 ... K:                        # K rollouts, done in parallel
        eps_i ~ N(0, Sigma)                 # shape (N, m)
        U_i   = clip(U + eps_i)             # respect actuator limits
        J_i   = 0
        for k = 0 ... N-1:
            x_{k+1} = f(x_k, U_i[k])
            J_i    += c(x_k, U_i[k]) + c_obs(x_k)
        J_i += phi(x_N)

    rho = min_i J_i                         # for numerical stability
    w_i = exp(-(J_i - rho)/lambda) / sum_j exp(-(J_j - rho)/lambda)

    U <- U + sum_i w_i * eps_i              # weighted update
    apply(U[0])                             # receding horizon
    U <- shift(U)                           # warm start for next step
```

Two implementation details hide in there. Clipping inside the rollout matters:
if you clip only at the end, you score trajectories that the actuators could
never have produced. And subtracting $$\rho = \min_i J_i$$ before exponentiating
changes nothing mathematically — the normalization cancels it — but it is the
difference between working code and a pile of `NaN`s.

## Example: merging into a moving gap

To see what MPPI actually does, consider a lane-merge. The ego vehicle starts in
the right lane and has to merge into a gap between two vehicles in the adjacent
lane — a gap that is itself moving.

### Problem setup

The ego vehicle is a five-state kinematic bicycle model,

$$
x = \begin{bmatrix} p_x & p_y & v & \theta & \delta \end{bmatrix}^{\top},
\qquad
u = \begin{bmatrix} \dot{v} & \dot{\delta} \end{bmatrix}^{\top},
$$

with dynamics

$$
\begin{aligned}
p_{x,k+1} &= p_{x,k} + v_k\cos(\theta_k)\,\Delta t,\\
p_{y,k+1} &= p_{y,k} + v_k\sin(\theta_k)\,\Delta t,\\
v_{k+1} &= v_k + \dot{v}_k\,\Delta t,\\
\theta_{k+1} &= \theta_k + \frac{v_k}{L}\tan(\delta_k)\,\Delta t,\\
\delta_{k+1} &= \delta_k + \dot{\delta}_k\,\Delta t.
\end{aligned}
$$

Note that steering angle $$\delta$$ is a *state*, not an input — the input is
steering rate. That keeps the commanded steering continuous, which matters when
the controller is stitching together random perturbations.

The objective is not to reach a fixed point. The target is the centre of a gap
between two vehicles, and that gap is moving forward, so the target position is
propagated forward inside each prediction horizon.

### Sampling

At each MPC step, MPPI starts from a nominal control sequence
$$U_{\mathrm{nom}}$$ and generates $$K$$ perturbed sequences

$$
U^{(i)} = U_{\mathrm{nom}} + \epsilon^{(i)},
\qquad
\epsilon^{(i)} \sim \mathcal{N}(0,\Sigma).
$$

Here $$K = 2000$$ and the horizon is $$N = 40$$ steps at $$\Delta t = 0.1\,$$s. Every
optimization step therefore considers two thousand possible futures over a four
second horizon. Each candidate is rolled out through the bicycle dynamics and the
resulting state trajectory is scored.

### Cost function

The vehicle is encouraged to track the moving target, hold the desired velocity
and heading, and avoid violent control changes:

$$
J_i = J_{\mathrm{tracking},i} + J_{\mathrm{heading},i} + J_{\mathrm{velocity},i}
    + J_{\mathrm{smoothness},i} + J_{\mathrm{collision},i} + J_{\mathrm{terminal},i}.
$$

The tracking term penalizes the distance between the predicted vehicle position
and the predicted position of the moving target, with lateral error weighted more
heavily than longitudinal error to encourage the merge.

Collision avoidance uses a bidirectional Minimum Signed Distance to Edges (MSDE)
cost. The ego vehicle and the obstacles are rectangular polygons, and the signed
distance is evaluated in both directions, with a safety margin folded in.
Trajectories that enter or crowd an obstacle pick up a large cost.

This is where sampling earns its keep: the collision cost is non-smooth and
non-convex, and MPPI never needs a gradient of it. It only needs to *evaluate*
it.

### From costs to weights

Once every trajectory has been scored, the costs become normalized weights:

$$
w_i = \frac{\exp\!\left(-\dfrac{J_i - \rho}{\lambda}\right)}
           {\sum_{j=1}^{K}\exp\!\left(-\dfrac{J_j - \rho}{\lambda}\right)},
\qquad \rho = \min_i J_i.
$$

Subtracting $$\rho$$ does not change the normalized weights; it only keeps the
exponentials in a sane numerical range. Low-cost trajectories get large weights,
while colliding trajectories contribute almost nothing to the update. The nominal
sequence is then updated as

$$
U_{\mathrm{new}} = \sum_{i=1}^{K} w_i U^{(i)},
$$

with a momentum term in the implementation blending the new estimate against the
previous nominal sequence.

### What the samples look like

![MPPI rollouts for the dynamic lane-merge scenario](/images/mppi_lane_merge.gif)

*The blue rectangle is the ego vehicle and the red rectangles are the
surrounding vehicles. The red cross is the moving target at the centre of the
gap. Light-blue curves are a subset of the sampled trajectories — MPPI evaluates
2000 rollouts per iteration, but only the best candidates are drawn to keep the
figure readable. The dashed curve is the plan produced by the updated nominal
control sequence.*

Watch the fan of samples: it is widest when the vehicle is uncommitted, and
collapses onto a single path once the merge is decided. The width of that fan is
the controller's uncertainty, made visible.

### Receding-horizon execution

After the update, MPPI applies only the first control input,
$$u_k = U_{\mathrm{new},0}$$. The vehicle is propagated forward one timestep, and
the remaining sequence is shifted and reused as the nominal sequence for the next
iteration. The full loop:

$$
\boxed{
\begin{array}{c}
\text{Sample control sequences}\\ \downarrow\\
\text{Roll out dynamics}\\ \downarrow\\
\text{Evaluate costs}\\ \downarrow\\
\text{Compute weights}\\ \downarrow\\
\text{Update control sequence}\\ \downarrow\\
\text{Apply first control}\\ \downarrow\\
\text{Shift horizon and repeat}
\end{array}}
$$

At every timestep the controller regenerates a fresh collection of possible
futures from the current state and the current positions of the moving
obstacles. Nothing is planned once and trusted; everything is re-derived.

## Wrapping up

MPPI buys three things: no gradients, so arbitrary non-smooth costs are fair
game; embarrassing parallelism, so the rollouts map cleanly onto a GPU; and a
dead-simple update rule. What it gives up is any guarantee — no constraint
satisfaction, no stability certificate, and a strong dependence on how well the
sampling distribution covers the useful part of control space.

That last weakness is what most of the follow-up work attacks: smoothing the
sampled sequences, adapting the covariance, warm-starting from a better prior, or
tightening the sampling around a nominal plan. Those variants are for another
post.