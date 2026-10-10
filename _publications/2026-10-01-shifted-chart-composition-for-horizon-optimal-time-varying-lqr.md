---
title: "Shifted-Chart Composition for Horizon-Optimal Time-Varying LQR"
collection: publications
category: manuscripts
permalink: /publication/2026-10-01-shifted-chart-composition-for-horizon-optimal-time-varying-lqr
excerpt: 'An inverse-coordinate composition for fast horizon-optimal time-varying LQR that handles positive semidefinite costs'
date: 2026-10-01
venue: 'Submitted to 2027 American Control Conference'
paperurl:
citation:
---
Horizon optimization in finite-horizon LQR treats the horizon \(T\) as a decision variable and adds a penalty \(w_T\) to the cost. Recent composition-based methods evaluate all candidate horizons in \(O(N n^3)\), instead of the \(O(N^2 n^3)\) of one Riccati recursion per horizon, but they track the inverse cost-to-go \(P^{-1}\) and therefore require every stage and terminal cost to be positive definite, whereas LQR requires only positive semidefiniteness. This excludes costs that leave some states unpenalized. In this work, we track instead the shifted inverse \(\hat{P}_k = (P_k + cI)^{-1}\), the inverse coordinate after a symplectic shear of the state–costate variables. Its domain contains every positive semidefinite Riccati matrix, and \(P_k = \hat{P}_k^{-1} - cI\) recovers the Riccati solution. We show that the shift can be absorbed into the stage coefficients of the inverse-coordinate composition, so the \(O(N n^3)\) search carries over, exact in exact arithmetic, subject to invertibility conditions. These hold for all small \(c\) when the dynamics contract the unpenalized directions, fail for at most finitely many \(c\) outside a degenerate case, and are certified per horizon at run time. In our experiments, diagonal loading \(Q_k + \epsilon I\) either left cost errors several orders of magnitude above those of the shifted chart or, for small fixed \(\epsilon\), selected the wrong horizon. The shifted chart with a fixed default \(c\) selected the correct horizon on all tested instances, reporting long horizons as uncertified in one stress case.

*Preprint not yet available — paper has been submitted and is under review.*
