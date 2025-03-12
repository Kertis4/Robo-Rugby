# RoboRugby 2025 - Competition Rules
## 1. Governing Principles
- **Disclaimer**: Nothing in these rules overrides the need for mutual respect, academic honesty, common sense, and the requirements of the UCD Student Code.
- **Gameplay**: In each 60-second match, two robots will autonomously compete against each other on the RoboRugby table, as available in the ENG 329 laboratory. The objective for each robot is to move balls into their scoring zone to score as many points as possible. A robot may also impede their opponent's scoring efforts, subject to some restrictions. At the end of each match, scores will be calculated using the scoring rules. A set of rules exists to resolve ties: every match must have a winner.
- **Balls**: At the start of a match, 19 balls, of four types, will be placed in designated positions on the game table. Each ball type has an associated points value.
- **Penalties**: Violation of any of the rules may result in a team forfeiting a match or being disqualified from the competition, at the discretion of the RoboRugby organizers.
- **Tournament**: The RoboRugby competition will consist of a single-elimination tournament. Each team will embark on a series of matches against other robots. After losing one match, a robot will be eliminated. The robot that wins the final match will be the overall winner of the competition.
- **Changes**: All rules are subject to change at the discretion of the RoboRugby organizers. Any new or modified rules will be notified to all participating teams.
- **Interpretation**: The interpretation of the rules will be decided by the RoboRugby organizers. All decisions of the RoboRugby organizers are final.

## 2. Gameplay
- **Time**: After the start, the robots have 60 seconds to compete and score points. At the end of 60 seconds, the robot must turn off electrical power to its actuators (motors and servos). Software will be provided to do this. A match may be terminated after less than 60 seconds if the organisers and both teams agree to this (for example, if both robots have stopped moving and are unlikely to resume).
- **Separation**: During a match, a robot may separate into at most three separate sections as part of its strategy. Robots may not be designed to, nor have a tendency to, break into more than three sections to gain strategic advantage.
- **Control**: During a match, a robot must be controlled solely by its onboard computer, using the software submitted at impounding.
- **Human Intervention**: During the match and scoring, the human contestants must remain at least 1 m from the table. They may not touch the table or a robot or otherwise interfere with the match or the scoring.
- **Unnecessary Roughness**: The objective in RoboRugby is to win a match by scoring more points than the opposing robot. A robot may be designed to impede or obstruct the opposing robot, but not to damage or destroy it.
  - A robot may make contact with the other robot, provided the intent is to impede the movement of the other robot or to prevent the robot from executing its strategy correctly. Contact intended to damage the other robot is not allowed.
  - A robot may shoot a projectile in any direction (including in the direction of the other robot), provided the main purpose of the projectile is to move balls or to obstruct the other robot. Projectiles intended to damage the other robot are not allowed. Projectiles that present a hazard to humans are forbidden.
  - Robots may not use battering-rams, hammers, or similarly destructive contrivances to damage the other robot.
- **Collateral Damage**: A robot may not be designed to damage or to attempt to damage the table or any part of the table. A robot that shows a tendency to cause such damage may be disqualified.
- **Safety**: Robots must not present a safety hazard to the competitors, organizers, or spectators. A robot that is deemed to present a hazard will be disqualified.

## 3. Scoring Calculation
- **Score**: The score that each robot receives is determined by the final state of the contest table after the match has ended. A team's score is calculated as the sum of the points values of all the balls within their designated scoring zone at match end. The team with the greater final score wins the match.
- **Completion**: The match ends when all robots and balls on the table come to rest after the 60-second play period has elapsed, or earlier if agreed.
- **Scoring Zones**: As indicated on the diagram, the scoring zones are the areas of grey at either end of the table.
- **Score**: The score of each robot depends solely on the summation of points values of balls in its scoring area at the end of the match.
- **Valid Scores**: For a ball to score, it must be in physical contact with the grey surface of the scoring area.
- **On the Lines**: Balls "on the lines" are scored as if in the scoring zone. A ball is "on the line" if it is touching one of the raised bumps, and is not touching the grey horizontal surface of the scoring zone.
- **Elevation**: Balls within the scoring zone, but resting on other objects, do not score unless such balls are also touching the grey horizontal surface of the scoring area or one of the raised bumps surrounding it.
- **Walls**: Balls touching the walls of the scoring area do not score, unless they are also touching the grey horizontal surface of the scoring area or one of the raised bumps surrounding it.
- **Ball Values**: The points value of each ball will be calculated as below:
  | Ball Colour | Number Available | Points Value |
  |-------------|-------------------|--------------|
  | White       | 12                | 1            |
  | Yellow      | 4                 | 2            |
  | Red         | 2                 | 4            |
  | Multi-coloured | 1               | 7            |

## 4. Robot Construction
- **Kit Parts**: The robot must be built only from the parts in the kit provided to each team, except when explicitly allowed by other rules. All kits contain broadly the same set of components, although some parts may be coloured differently in different kits. If a team feels they are lacking a part that should be in their kit, this should be communicated to the module organisers at the earliest opportunity.
- **Assembly**: The structure of the robot must consist of LEGO parts, held together by normal LEGO studs, pins, axles, etc. LEGO parts may not be modified in any way. LEGO parts may not be joined by adhesive.
- **Additional Parts**: Insulating tape, sticky-pads, and rubber bands will be available in the laboratory. These items may be used only in reasonable quantities, and only for the specific purposes listed below. These additional parts may not be used to increase the rigidity of the robot.
  - Rubber bands may be used for purposes that require their flexibility or elasticity.
  - Insulating tape may be used in small quantities for shielding or mounting sensors or other electronic components. It may be used for its adhesive properties, to make a temporary connection between a part of the robot and an external object. It may also be used for its smooth surface, to reduce friction. It may not be used to bind LEGO parts together.
  - Sticky-pads may be used only in small pieces to mount sensors on the robot.
  - Rubber bands or insulating tape may be applied to LEGO wheels and treads to alter their coefficient of friction. No other material may be applied to wheels or treads.
- **Lubricants**: No lubricants of any kind are permitted.
- **Sections**: A section is a group of connected parts (including a single part). A robot may be constructed as up to three separate sections, but at the start of a match, each section must be in contact with at least one other section of the robot.
- **Dimensions**: In its starting configuration, the robot's dimensions must not exceed 400 mm by 320 mm in plan and 300 mm in height. A cuboid measuring gauge is available to check the size of the robot, but the robot must satisfy this rule without being artificially constrained by the measuring gauge or any other means. For storage and transport, the robot must fit completely within the plastic storage box provided, with the lid of the box closed.
- **Identification**: To facilitate identification of the robot, the number on the Arduino unit should be visible. If necessary, an alternative label may be agreed with the organizers.
- **Start-up**: To facilitate the start-up sequence in matches, the large white buttons on the LCD shield must be accessible.

## 5. Impounding
- **Pre-Competition**: Before the competition, all the robots and their programs will be impounded. The exact time of impounding will be confirmed in the lead up to the competition.
- **Alterations**: After impounding, the structure of the robot may not be altered in any way. Minor repairs may be made and batteries may be charged between matches as time permits.
- **Program Changes**: After impounding, the robot's program may not be altered in any way. Contestants are not permitted to download any program to the robot from any other device after impounding.

## 6. Start-up
- **Set-up Time**: Before each match, the contestants will have up to 60 seconds to set up their robot. Separate sections of the robot must be placed in contact, moving elements of the robot may be set to desired positions, elastic bands may be stretched, etc. Pre-programmed routines may be run on the Arduino to adjust parameters in the competition software, to choose strategy options, to calibrate sensors, to adjust for battery condition, etc. No other alterations to the robot are allowed: the robot must start every match in the same mechanical and structural state.
- **Starting Area**: The starting zone for each team is shown on the table diagram. The interiors of these zones are the only parts of the table's surface that a robot may be in contact with at the start of the match.
- **Starting Position**: Before the end of the set-up time, the contestants must set up their robot in the North or South starting area, as stipulated by the organizers. The contestants may position the robot anywhere in the starting area, in any orientation, provided that the constraints of the foregoing rule b. are met. A robot may not be in contact with the tape-lines and ridges which delineate the zone's border at the start of the match.
- **Start-up Software**: Software to implement a synchronised start-up procedure for the two robots will be provided by the organizers and must be used as instructed. This software will also shut down the robot after 60 seconds of play.

## 7. Tie-breaking
- **Criteria Hierarchy**: In each match, the winning robot is decided using the rules below. The first rule that distinguishes between the robots decides the winner.
  1. The robot that scored the most points.
  2. The robot that scored the higher aggregate number of balls.
  3. The robot that scored the multi-coloured ball.
  4. The robot that scored more red balls.
  5. The robot that scored more yellow balls.
  6. The robot that scored more white balls.
  7. The robot that has the multi-coloured ball on the same side as its scoring area.
  8. The robot that has more red balls on the same side as its scoring area.
  9. The robot that has more yellow balls on the same side as its scoring area.
  10. The robot that has more white balls on the same side as its scoring area.
  11. The robot that moved some part of itself, under its own power, at some time during the match.
  12. The team with fewer members (e.g., a two-person team beats a three-person team).
  13. Rock-paper-scissors.
- **Sides**: The table is divided into a North "side" and a South "side" by a lateral white line across the centre of the table, parallel to the short side of the table. This lateral white line is not part of either side.
- 
