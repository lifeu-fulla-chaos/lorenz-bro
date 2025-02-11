# Phase 2 - Meetings

## With mentor

-   ### 30th Dec 2024
    
    Told us to meet with seniors and learn all that they've done. Come on 2nd and 3rd Jan and learn their implementation.

-   ### 13th Jan 2025

    Showed her backstepping and synchronization code and graphs. Were informed to learn how the backstepping code works and explain it to her. 

-   ### 21st Jan 2025

    Showed her new and improved backstepping, transmission of a single text message. Told her we want to attempt to improve synchronization accuracy with ML. Were give the green light. 

-   ### 28th Jan 2025
    Presented her with all the problems, suggested usage of neuralODE. Told us to use non autonomous systems to see if it works with it. Pelvan system

    ### 6th Feb 2025
    Present her with further problems. Approved the idea of using acks and solving for shorter periods of time to simulate fewer states at a time. Non autonomous system can have input anywhere. Try something to coordinate. 

## Team meetings

-   ### 2nd Jan 2025

    Met senior team. Nilankh showed us their work. They used a NeuralODE to simulate chaotic systems. Sliding mode controller to synchronize. He sent us Julia code for basins of attraction, bifurcation. Julia setup was done.

-   ### 8th Jan 2025

    **SS** tried to run and understand basin of attraction. Wrote the code for synchronization and plotted trajectories.

    **AV** tried to run and understand bifrucation.

    **SK** tried to run master slave on different ports and synchronize over sockets.

-   ### 12th Jan 2025

    Online meet to explain **SM** everything done so far and bring a little clarity to the rest of the team.

-   ###  15th Jan 2025

    **SS, SM, AV** tried to follow the textbook and a few papers to figure out the backstepping equation. 60% done

-   ### 19th Jan 2025

    **SS** modelled the system using `ODEProblem` with `Tsit5` solver instead of euler integration. 

    **SM and AV** successfully completed the backstepping eqation derivation and the equations were implemented and ran successfully. 

    **SK** wrote the code to send a text message between the master and the slave using the final master state. Shortcoming of the system was highlighted. 

-   ### 25th Jan 2025
    **AV** derived backstepping equation for rossler system using rossler as slave, lorenz as master
    **SM and SS** created the data for training. Figured out new roadblocks about coordination/synchronization
    **SK** worked on having two asnychronous systems and sending message using that.

-   ### 26th Jan 2025
    **AV** derived backstepping eqation for rossler system using lorenz as slave, rossler as master
    **SM and SS** got code for Tsit5 solver, threshold logic to know when synchronization has occured
    **SK** continued same task

    #### Roadblocks
    -   coordination how to do - slave has moved to new time step by the time master encrypts a message andd sends it. need to figure if coordination or synchronization issue
    -   tsit5 solver - how does it work, how to use it to emit a new state when needed

    ### 29th Jan 2025
    Attempt to learn NeuralODE. Huge bust. No one could figure. 

-   ### 5th Feb 2025
    - **SK** attempted to encrypt a message and send it and once again, the highlighted issue was the coordination between slave and master. 
    - **SM and AV** tried to run a non autonomous system and had doubts about which side can have the extra input
    - Ideation of using acks for coordination between slave and master. 

-   ### 9th Feb 2025
    - Coming up with the design for coordination. 
    - send 50 packets, synchronize. 
    - then send 30 at a time and use a seed to randomly select one of the 30 to encrypt.
    - use acks
    - function to genrate states with modes
    - sockets to communicate

    ### 10th Feb 2025
    - Bring **SK** up to date
    - Attempt to make the describe system. Failed. 

