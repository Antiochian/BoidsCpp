# BoidsCpp
A C++ rewrite of my agent-based boid model

This was originally supposed to be a straight port of my Python Boids project into C++ to improve the framerate, but during the process of making this I had several epiphanies and ended up completely reworking the algorithm into a much better-executing and better-performing form. Amazing!

One of the benefits of the programs newfound robustness is that I could implement dynamic addition of boids - simply click anywhere on the screen and a new boid will be created with a random starting velocity. You can also *hold* right click to enter "debug mode", which will cause one boid's vision range to be highlighed in red.

Gallery
-------
| demo video | adding new boids |
|:---:|:---:|
|![](demo1.gif)|![](demo2.gif)
| debug mode | reset button |
|:---:|:---:|
|![](demo3.gif)|![](demo4.gif)

What is it?
-------

The approach used is to simulate boids here is taken from [this 1987 research paper](https://www.red3d.com/cwr/papers/1987/boids.html).
This is a super useful and beautifully simple technique for modelling crowd behaviour, with immediate canonical applications to simulating:
 - flocks of birds
 - shoals of fish
 - packs of animals
 - hordes of zombies
 
 
