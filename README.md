# virtual-environment
*Virtual haptic interface environment*

This repository contains matlab code written as part of a robotics course at UPenn, coauthored by JoAna Smith and Hung-Tang Ko. The objective is to develop a virtual environment for a haptic interface. Haptics is the branch of robotics where a user can experience an artificial sense of touch and texture by interacting in a virtual space via a kinesthetic communication device that gives mechanical feedback to the user. The SensAble Phantom haptic device would be optimal for interacting with the given program.

# the environment
We chose to develop a simple game similar to squash, where a ball could be poked and pushed by the user. The user can only operate in the limit of one half of the domain. There's also a gravity on/off button that the user could press to pull both the user and the ball towards the button, creating an additional challenge to play against. Other features of the environment include a sense of viscous drag, and a sense of rough texutre on the walls of the domain.

# user interaction
In case a haptic interface robot is not available, the user can use keyboard controls: there's no sense of haptics, but the visual interaction is still possible. controls are as follows:
I-In
O-Out
U/↑-Up
D/↓-Down
L/←-Left
R/→-Right

*virtual_environment_team239_FINAL.m is to be run*
