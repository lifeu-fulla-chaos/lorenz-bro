# Phase 2 - Meetings

## With mentor

-   ### 30th Dec 2024
    
    Told us to meet with seniors and learn all that they've done. Come on 2nd and 3rd Jan and learn their implementation.

-   ### 13th Jan 2025

    Showed her backstepping and synchronization code and graphs. Were informed to learn how the backstepping code works and explain it to her. 

-   ### 21st Jan 2025

    Showed her new and improved backstepping, transmission of a single text message. Told her we want to attempt to improve synchronization accuracy with ML. Were give the green light. 

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






