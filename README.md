# Neural Dynamics and Synaptic Plasticity in Simple Networks Drive Lévy Flight Foraging and Obstacle Avoidance Behaviors
<br>  
<div align="center">
   <img width="60%" height="60%" src="Figure/Fig.0.4.svg">
</div>
<br>  
&emsp;Animal foraging behavior is essential for survival. This complex behavior involves a combination of local search--seeking food nearby and global search--exploring large areas for new food resources. These strategies can be adaptively alternated. During exploration, animals also navigate obstacles. Several studies have been focused on developing bio-inspired exploration mechanisms for artificial agents, with Lévy flight emerging as one of the most effective methods. However, existing Lévy models rely on mathematical representations of Lévy distributions, which are difficult to directly relate to the biological neural systems of animals. From this perspective, we propose an efficient and adaptive neural exploration control system synthesized from a small recurrent neural network. This network, characterized by its synaptic weights and adaptive inputs, exhibits neural dynamics similar to the discrete-time Lorenz attractor. The network's output provides the baseline input signal for post-processing through an adaptive bias thresholding mechanism, which generates exploration commands. These commands enable adaptive exploration behaviors that incorporate both local and global search strategies.
Additionally, to ensure safe exploration in obstacle-filled environments, we integrate an adaptive neural obstacle avoidance control into the system. We assessed the performance of the proposed system in both open and closed environments using simulated and real flying robots. The resulting flight trajectories demonstrate animal-like foraging behaviors, including local and global search strategies similar to Lévy flight, along with adaptive obstacle avoidance capabilities.
<br>  
<div align="center">
   <img width="60%" height="60%" src="Figure/Fig.1.3.svg">
</div>
