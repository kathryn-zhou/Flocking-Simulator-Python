# A demonstration of flocking behavior in distributed robotic systems using matplotlib animation


It has long been known to roboticists and bird watchers that complex behaviors in groups of autonomous agents can emerge from a combination of primitive behaviors exhibited by each agent independently. Dr. Maja J. Mataric of MIT’s Artificial Intelligence Lab has described several of these primitive behaviors in her work. A few of these are listed below:

* **safe wandering**- minimizes collisions between agents and environment
* **following**- minimizes interference by structuring movement of any two agents
* **aggregation**- gathers the agents
* **dispersion**- dissipates the agents
* **homing**- enables the agent to proceed to a particular location

Note again that every agent exhibits these behaviors only locally. But if all agents exhibit these behaviors, the group can collectively demonstrate more complex behaviors. An example of such a complex behaviour is flocking, also referred to as swarming in the context of insects. According to Mataric, flocking is a combination of *safe wandering, aggregation, dispersion* and *homing*. Look at the following picture for example.

<img src="flock.jpg" width="500" border="1000">

Each bird (autonomous agent) in the flock is directed by a desire to stay close to the other birds it can see, to try its best to not run into them, and to move towards the desired location. These three behaviours, scaled, gives rise to the robust and well organized flock.

In this project, I will try to demonstrate this behavior in a group of robots through a simple python simulation.

