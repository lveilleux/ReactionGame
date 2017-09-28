##Reaction Timing Game built for the FRDM-KL46Z board  
  
This is a reaction game that uses of the touch sensor (TSI) built into the KL46 board. The game uses the UART0, PIT, and TSI
to run games of 10 rounds testing the users reaction time from when the green LED lights to when they touch the sensor.
A score is given based on the time taken to touch the sensor. A 'random' amount of time occurs before the green LED turns back on
at the start of each round. If a touch is registered before the LED turns on, max points are awarded for that round.  
**Lowest score wins.**  
  
This game was built for RIT's CMPE-250 Assembly Programming course from Fall Semester 2016. Code files were built from templates taken from that course.  
Code for initialization was taken from a source on the web (_I don't remember where at this point in time._) All other program and functionality code was written by me.  