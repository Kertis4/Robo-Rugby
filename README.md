# Robo-Rugby
The repository for my team's contribution to UCD's 2025 Robo-Rugby Competition
# Team 10 Strategy Report: Scorpio
## Overview of RoboRugby 2025

RoboRugby is a robotics competition where autonomous robots compete to score the most points within a 60-second match. Each robot must strategically collect and move balls into their designated scoring zone while also having the option to obstruct their opponent’s progress. The game rewards both offensive and defensive strategies, requiring teams to carefully balance scoring efficiency with disruption tactics.

The competition follows a single-elimination tournament format, meaning each match is critical; losing a single round results in elimination. With 19 balls of varying point values positioned on the table at the start of each match, teams must develop creative and efficient ways to maximize their score while minimizing their opponent’s.



## Created Strategies

### 1. Charge Down

This is our most basic strategy, taking advantage of the one ball down the line of the starting area. This will result in a score of two in a perfect simulation.

- **Implementation**: Simple to set up by making the right side of the bot move faster, effectively pushing along the wall.
- **Drawbacks**: Will be beaten by teams who can score more than one ball. It is built on the assumption that most teams don’t score one ball.

### 2. Blockade

Another primitive strategy, where the bot drives to the opponent's scoring box, removes the ball, and sits there.

- **Implementation**: Requires a lifter arm or crab-like attachment to remove the ball from the scoring zone.
- **Advantages**: Can beat more advanced bots if they are slow. However, there is a risk of damage from fast robots.

### 3. Brachistochrone Bot

A more advanced version of the charge-down bot using a curve to go through the middle and then into the scoring area.

- **Implementation**: Requires PI control and a distance counter to turn at the right time.
- **Drawbacks**: Loses speed and may lose to the blockade bot. However, it has a higher scoring potential than charge-down.

### 4. Blockade V2

Our most ambitious strategy, using the rule of splitting into three pieces to blockade the opponent’s goal.

- **Implementation**: Requires additional hardware like a plate dropper and complex software.
- **Challenges**: Nearly impossible to implement due to complexity.

## Critical Evaluation

When evaluating strategies, consider:
- **Implementation Difficulty**: The complexity of design and code.
- **Potential Weaknesses**: How opponents might exploit the strategy.

A balanced strategy that can adapt to various opponents is crucial for success.

## Candidate Strategy

We will use a combination of Charge Down, Blockade, and Brachistochrone Bot strategies:
- **Charge Down** linked to button 1.
- **Blockade** linked to button 2.
- **Brachistochrone Bot** linked to button 3.
- **Adaptability**: Choose the strategy based on the opponent’s playstyle.

## Robot Development

### Software

To execute these strategies, the robot must perform basic tasks:
- **Moving in a Straight Line**: Requires sensor information and a compass.
- **Changing Direction**: Needs to handle the 0/360 boundary.
- **Position Tracking**: Essential for complex strategies like Blockade V2.

### Hardware

- **Gearing**: Low gearing for speed, suitable for Blockade and Brachistochrone Bot.
- **Ball Catcher**: Simple design sufficient for current strategies.
- **Future Plans**: Add distance tracker and crash pads for durability.

## Conclusion

Throughout this report, we have explored multiple strategies for RoboRugby 2025. By integrating Charge Down, Blockade, and Brachistochrone Bot into a single adaptable system, we maximize our chances of success in different match scenarios. Moving forward, we will refine our robot’s navigation system, optimize PI control, and conduct extensive testing to ensure consistency across strategies.



