---
layout: archive
title: "CV"
permalink: /cv/
author_profile: true
redirect_from:
  - /resume
---

{% include base_path %}

Education
======
* M.S. by Research  Control Lab, Robotics Research Centre, IIIT Hyderabad, 2025-Present
* B.E. in ECE, BITS Pilani KK Birls Goa Campus, 2021-2025

Work experience
======

* Spring 2025: Research Intern
  * Robotics Research Centre, IIIT-H
  * Duties included: Designing a Fault-tolerant Control for quadrotor, writing an RTOS on Raspberry Pi Pico W
  * Supervisor: Dr. Harikumar Kandath

* Summer 2023: Research Intern
  * National Atmospheric Research Laboratory
  

Publications
======
  <ul>{% for post in site.publications reversed %}
    {% include archive-single-cv.html %}
  {% endfor %}</ul> 
  

  
Teaching
======

* Taught a lecture on State-Space controls in RRC Summer School 2026
* Teaching Assistant for Systems Thinking course  Monsoon 2026

Projects
======

* [Convex Polygon aware collision avoidance with MPC and MPPI for dynamic scenes](https://github.com/viswasdinesh/polygon-mpc)
  * Model Predictive Control and MPPI (Model Predictive Path Integral) control for polygon-based systems, with geometry, simulation, and visualization modules.
  * See related blog post: [MPPI: Model Predictive Control by Sampling]({{ base_path }}/posts/2026/08/mppi-sampling-based-mpc/)

* A simple ROS2 package for Slope Detection with realsense
