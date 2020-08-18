# BoidsCpp
Swarming agents, visualised.

This is a much-improved C++ version of my agent-based boid model from [a while ago](https://github.com/Antiochian/boids).
This was originally supposed to be a straight port of my Python Boids project into C++ to improve the framerate, but during the process of making this I had several epiphanies and ended up completely reworking the algorithm into a much better-executing and better-performing form. Amazing!

One of the benefits of the programs newfound robustness is that I was able to easily implement dynamic addition/deletion of boids - simply click anywhere on the screen and a new boid will be created with a random starting velocity. You can also *hold* right click to enter "debug mode", which will cause one boid's vision range to be highlighed in red, or hit space to reset everything.

Gallery
-------
| demo video | adding new boids (left click) |
|:---:|:---:|
|![](demo1.gif)|![](demo2.gif)
| debug trace mode (right click)| reset function (spacebar) |
|![](demo3.gif)|![](demo4.gif)

What is it?
-------

The approach used is to simulate boids here is taken from [this 1987 research paper](https://www.red3d.com/cwr/papers/1987/boids.html).
This is a super useful and beautifully simple technique for modelling crowd behaviour, with immediate canonical applications to simulating:
 - flocks of birds
 - shoals of fish
 - packs of animals
 - hordes of zombies
 
 This program uses the excellent "olcPixelGameEngine" library by [David Barr](https://github.com/OneLoneCoder) to handle all the visuals.
