Overview
======

Path planning means that the robot automatically plans a path from the starting point to the goal point in the corresponding area, in which it needs to ensure that no collision occurs and the cost of pathfinding is low. Currently, the main problems of path planning are the difficulty of environment modeling, slow convergence of the algorithm, and easy to fall into the local optimal solution, and the usual solution is to improve the structure of the algorithm, increase the adaptive ability of the algorithm parameters, or combine the algorithms to optimize the algorithm performance. 

This repository implements some common path planning algorithms used in robotics, including global path planning based on a priori complete information and local path planning based on sensor information. We designed animation for each algorithm to display the running process. 

In addition, the project is based on [zhm-real/PathPlanning](https://github.com/zhm-real/PathPlanning) secondary organization, if there is any infringement, please contact the author, welcome to the big shot guidance. 

[TOC]



# Global Path Planning

## Search-based Planning

### BFS & DFS

<div align=right>
<table>
  <tr>
    <td><img src=".\Search-based Planning\map\gif\BFS.gif" alt="BFS" width="400"/></a></td>
    <td><img src=".\Search-based Planning\map\gif\DFS.gif" alt="DFS" width="400"/></a></td>
  </tr>
</table>
</div>

### Best-First & Dijkstra

<div align=right>
<table>
  <tr>
    <td><img src=".\Search-based Planning\map\gif\Best_First.gif" alt="Best_First" width="400"/></a></td>
    <td><img src=".\Search-based Planning\map\gif\Dijkstra.gif" alt="Dijkstra" width="400"/></a></td>
  </tr>
</table>
</div>

### A* and A* Variants

<div align=right>
<table>
  <tr>
    <td><img src=".\Search-based Planning\map\gif\Astar.gif" alt="Astar" width="400"/></a></td>
    <td><img src=".\Search-based Planning\map\gif\Bidirectional_Astar.gif" alt="Bidirectional_Astar" width="400"/></a></td>
  </tr>
  <tr>
    <td><img src=".\Search-based Planning\map\gif\Repeated_Astar.gif" alt="Repeated_Astar" width="400"/></a></td>
    <td><img src=".\Search-based Planning\map\gif\ARAstar.gif" alt="ARAstar" width="400"/></a></td>
  </tr>
  <tr>
    <td><img src=".\Search-based Planning\map\gif\LRTAstar.gif" alt="LRTAstar" width="400"/></a></td>
    <td><img src=".\Search-based Planning\map\gif\RTaAstar.gif" alt="RTaAstar" width="400"/></a></td>
  </tr>
</table>
</div>
 D* and D* Variants

<div align=right>
<table>
  <tr>
    <td><img src=".\Search-based Planning\map\gif\Dstar.gif" alt="Dstar" width="400"/></a></td>
    <td><img src=".\Search-based Planning\map\gif\1.gif" alt="1" width="400"/></a></td>
  </tr>
</table>
</div>




## Group Intelligence Optimization

genetic algorithm ()
ACO algorithm ()
particle swarm algorithm ()

## Sampling-based Planning


## Reinforcement Learning

# Local Path Planning





# Reference

## Search-based Planning

- [A*: ](https://ieeexplore.ieee.org/document/4082128)A Formal Basis for the heuristic Determination of Minimum Cost Paths
- [Learning Real-Time A*: ](https://arxiv.org/pdf/1110.4076.pdf)Learning in Real-Time Search: A Unifying Framework
- [Real-Time Adaptive A*: ](http://idm-lab.org/bib/abstracts/papers/aamas06.pdf)Real-Time Adaptive A*
- [Lifelong Planning A*: ](https://www.cs.cmu.edu/~maxim/files/aij04.pdf)Lifelong Planning A*
- [Anytime Repairing A*: ](https://papers.nips.cc/paper/2382-ara-anytime-a-with-provable-bounds-on-sub-optimality.pdf)ARA*: Anytime A* with Provable Bounds on Sub-Optimality
- [D*: ](http://web.mit.edu/16.412j/www/html/papers/original_dstar_icra94.pdf)Optimal and Efficient Path Planning for Partially-Known Environments
- [D* Lite: ](http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf)D* Lite
- [Field D*: ](http://robots.stanford.edu/isrr-papers/draft/stentz.pdf)Field D*: An Interpolation-based Path Planner and Replanner
- [Anytime D*: ](http://www.cs.cmu.edu/~ggordon/likhachev-etal.anytime-dstar.pdf)Anytime Dynamic A*: An Anytime, Replanning Algorithm
- [Focussed D*: ](http://robotics.caltech.edu/~jwb/courses/ME132/handouts/Dstar_ijcai95.pdf)The Focussed D* Algorithm for Real-Time Replanning
- [Potential Field, ](https://journals.sagepub.com/doi/abs/10.1177/027836498600500106)[[PPT\]: ](https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf)Real-Time Obstacle Avoidance for Manipulators and Mobile Robots
- [Hybrid A*: ](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf)Practical Search Techniques in Path Planning for Autonomous Driving

## Sampling-based Planning

- [RRT: ](http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf)Rapidly-Exploring Random Trees: A New Tool for Path Planning
- [RRT-Connect: ](http://www-cgi.cs.cmu.edu/afs/cs/academic/class/15494-s12/readings/kuffner_icra2000.pdf)RRT-Connect: An Efficient Approach to Single-Query Path Planning
- [Extended-RRT: ](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.1.7617&rep=rep1&type=pdf)Real-Time Randomized Path Planning for Robot Navigation
- [Dynamic-RRT: ](https://www.ri.cmu.edu/pub_files/pub4/ferguson_david_2006_2/ferguson_david_2006_2.pdf)Replanning with RRTs
- [RRT*: ](https://journals.sagepub.com/doi/abs/10.1177/0278364911406761)Sampling-based algorithms for optimal motion planning
- [Anytime-RRT*: ](https://dspace.mit.edu/handle/1721.1/63170)Anytime Motion Planning using the RRT*
- [Closed-loop RRT* (CL-RRT*): ](http://acl.mit.edu/papers/KuwataTCST09.pdf)Real-time Motion Planning with Applications to Autonomous Urban Driving
- [Spline-RRT*: ](https://ieeexplore.ieee.org/abstract/document/6987895?casa_token=B9GUwVDbbncAAAAA:DWscGFLIa97ptgH7NpUQUL0A2ModiiBDBGklk1z7aDjI11Kyfzo8rpuFstdYcjOofJfCjR-mNw)Optimal path planning based on spline-RRT* for fixed-wing UAVs operating in three-dimensional environments
- [LQR-RRT*: ](https://lis.csail.mit.edu/pubs/perez-icra12.pdf)Optimal Sampling-Based Motion Planning with Automatically Derived Extension Heuristics
- [RRT#: ](http://dcsl.gatech.edu/papers/icra13.pdf)Use of Relaxation Methods in Sampling-Based Algorithms for Optimal Motion Planning
- [RRT*-Smart: ](http://save.seecs.nust.edu.pk/pubs/ICMA2012.pdf)Rapid convergence implementation of RRT* towards optimal solution
- [Informed RRT*: ](https://arxiv.org/abs/1404.2334)Optimal Sampling-based Path Planning Focused via Direct Sampling of an Admissible Ellipsoidal heuristic
- [Fast Marching Trees (FMT*): ](https://arxiv.org/abs/1306.3532)a Fast Marching Sampling-Based Method for Optimal Motion Planning in Many Dimensions
- [Motion Planning using Lower Bounds (MPLB): ](https://ieeexplore.ieee.org/document/7139773)Asymptotically-optimal Motion Planning using lower bounds on cost
- [Batch Informed Trees (BIT*): ](https://arxiv.org/abs/1405.5848)Sampling-based Optimal Planning via the Heuristically Guided Search of Implicit Random Geometric Graphs
- [Advanced Batch Informed Trees (ABIT*): ](https://arxiv.org/abs/2002.06589)Sampling-Based Planning with Advanced Graph-Search Techniques ((ICRA) 2020)
- [Adaptively Informed Trees (AIT*): ](https://arxiv.org/abs/2002.06599)Fast Asymptotically Optimal Path Planning through Adaptive Heuristics ((ICRA) 2020)

