---
title: "Matrix Calculus for Roboticists - 1"
date: 2026-08-28
permalink: /posts/2026/08/matrix-calculus/
excerpt: "Higher-dimensional Jacobians using Kronecker Product"
tags:
  - Calculus
  - Linear Algebra
---

How do you take the derivative of a function? 

If you have taken a basic calculus course, you would say, "Compute the slope of tangent line at (x,f(x))". Indeed! The core of it is something called "linearization", which is a widely used term in robotics and we will shortly see the reason why it is called that. But what happens when the function is vector-valued? Go a step further, what if it is a matrix function? 

Generally speaking, how do you predict change in output with respect to input for a system?

Consider a system with $f(x): \mathbb{R}^n \rightarrow \mathbb{R}^m$

For scalar-valued function, or Multi-Input Single-Output System, we have m = 1

From Taylor Series expansion, 

$$
f(x+\Delta x)
\approx
f(x)+ \frac{\partial f}{\partial x} \Delta x 
+
\frac{1}{2}\Delta x^T \frac{\partial^2 f}{\partial x^2} \Delta x
$$

Where the dimensions of $\frac{\partial f}{\partial x}$ is $\mathbb{R^{1xn}}$ and of $\frac{\partial^2 f}{\partial x^2}$ is $\mathbb{R^{nxn}}$

Watch what happens when m > 1 

$$
f(x+\Delta x)
\approx
f(x)+ \frac{\partial f}{\partial x} \Delta x 
+
\frac{1}{2}(\frac{\partial}{\partial x}[\frac{\partial f}{\partial x} \Delta x])\Delta x
$$

Where the dimensions of $\frac{\partial f}{\partial x}$ is $\mathbb{R^{mxn}}$, but the Hessian term looks nasty!! This is because it is and it takes the form of something called a 3rd rank Tensor. 

### Third-order tensors

Consider $f: \mathbb{R}^2 \rightarrow \mathbb{R}^2$ with $f(x) = \begin{bmatrix} f_1(x) \\ f_2(x)\end{bmatrix}$ and $x = \begin{bmatrix} x_1 \\ x_2 \end{bmatrix}$.

The Jacobian is a familiar object, indexed by *output* $i$ and *input* $j$:

$$
J = \frac{\partial f}{\partial x} =
\begin{bmatrix}
\dfrac{\partial f_1}{\partial x_1} & \dfrac{\partial f_1}{\partial x_2} \\[1.2em]
\dfrac{\partial f_2}{\partial x_1} & \dfrac{\partial f_2}{\partial x_2}
\end{bmatrix}
\in \mathbb{R}^{2 \times 2}
$$

Differentiate once more and every entry of $J$ spawns two derivatives. We now need three
indices — output $i$, and the two input directions $j$ and $k$:

$$
\mathcal{T}_{ijk} = \frac{\partial^2 f_i}{\partial x_j \, \partial x_k}
$$

This is a $2 \times 2 \times 2$ object: a **third-order tensor** (often called rank-3, though
"rank" already means something else for matrices). There is no way to write it as a single
matrix without choosing a flattening — which is exactly the problem the rest of this series
is about.

The useful way to picture it is as a **stack of slices**. Fix the output index $i$, and what
remains is an ordinary Hessian:

$$
H_i = \frac{\partial^2 f_i}{\partial x \, \partial x^\top} \in \mathbb{R}^{2 \times 2}
$$

One Hessian per output. The tensor is the stack $\{H_1, H_2\}$.

#### A concrete example

Take the forward kinematics of a two-link planar arm with unit link lengths, where $x_1, x_2$
are joint angles and $f$ is the end-effector position:

$$
f(x) =
\begin{bmatrix}
\cos x_1 + \cos(x_1 + x_2) \\
\sin x_1 + \sin(x_1 + x_2)
\end{bmatrix}
$$

Writing $c_1 = \cos x_1$, $s_1 = \sin x_1$, $c_{12} = \cos(x_1 + x_2)$, $s_{12} = \sin(x_1 + x_2)$,
the Jacobian is the manipulator Jacobian you have already met:

$$
J =
\begin{bmatrix}
-s_1 - s_{12} & -s_{12} \\
 c_1 + c_{12} &  c_{12}
\end{bmatrix}
$$

Differentiating again gives one Hessian slice per output coordinate:

$$
H_1 =
\begin{bmatrix}
-c_1 - c_{12} & -c_{12} \\
-c_{12}       & -c_{12}
\end{bmatrix}
\qquad
H_2 =
\begin{bmatrix}
-s_1 - s_{12} & -s_{12} \\
-s_{12}       & -s_{12}
\end{bmatrix}
$$

Two observations worth carrying forward.

First, each slice is **symmetric**, since $\partial^2 f_i / \partial x_j \partial x_k =
\partial^2 f_i / \partial x_k \partial x_j$. The tensor is symmetric in its last two indices —
but not in all three, since swapping $i$ with $j$ mixes an output index with an input index.

Second, in practice you almost never want the tensor itself. You want it **contracted** against
a vector $\lambda \in \mathbb{R}^2$:

$$
\sum_i \lambda_i H_i \;=\; \lambda_1 H_1 + \lambda_2 H_2 \;\in\; \mathbb{R}^{2 \times 2}
$$

which is an ordinary matrix again. Stack the slices vertically into
$\mathcal{H} = \begin{bmatrix} H_1 \\ H_2 \end{bmatrix} \in \mathbb{R}^{4 \times 2}$
and that contraction has a compact form:

$$
\lambda_1 H_1 + \lambda_2 H_2 = (\lambda^\top \otimes I_2)\,\mathcal{H}
$$

That $\otimes$ is the Kronecker product, and this identity is where we are headed. In
differential dynamic programming, $\lambda$ is the value function gradient $V_x$ and this exact
contraction is the second-order term that iLQR throws away.

## Derivative as linear operation



## Gradient and Jacobian



### Side note(in a box): Chain rule and product rule for matrix functions
definition and a couple of examples


## Jacobian of Matrix functions

### Vec trick

### Kronecker product
