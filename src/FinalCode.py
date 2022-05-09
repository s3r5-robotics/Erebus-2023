from controller import Robot
import sys
import cv2 as cv

sys.path.append(r"C:\\Users\\LER\\Documents\\Programming\\CoSpace-2022-new\\Erebus-2022\\src\\")
from AbstractionLayer import AbstractionLayer  # li
from StateMachines import StateManager  # li


timeStep = 16 * 2
stMg = StateManager("init")
r = AbstractionLayer()

isOptimised = cv.useOptimized()

# While the simulation is running
while r.doLoop():
    #e1 = cv.getTickCount()
    # Update the robot
    r.update()
    #print("rotation: " + str(r.rotation))
    #print("position: " + str(r.position))
    #print("State:", stMg.state)


    if not stMg.checkState("init"):
        if r.isEnded():
            r.seqResetSequence()
            stMg.changeState("end")

        elif r.isVictims():
            r.seqResetSequence()
            stMg.changeState("victim")



    if stMg.checkState("init"):
        if r.calibrate():
            stMg.changeState("followBest")

    if stMg.checkState("stop"):
        r.seqMg.startSequence()
        r.seqMoveWheels(0, 0)

    if stMg.checkState("moveForward"):
        r.seqMg.startSequence()
        r.seqMoveWheels(0.5, -0.5)
        r.seqDelaySec(0.1)
        r.seqMoveWheels(-0.5, 0.5)
        r.seqDelaySec(0.1)
        r.seqResetSequence()


    if stMg.checkState("followBest"):
        r.seqMg.startSequence()
        bestPos = r.getBestPos()
        if bestPos is not None:
            r.seqMoveToCoords(bestPos)
        r.seqResetSequence()

        if r.isTrap:
            r.seqResetSequence()
            stMg.changeState("hole")


    if stMg.checkState("hole"):
        r.seqMg.startSequence()
        r.seqMoveWheels(-0.5, -0.5)
        r.seqDelaySec(0.5)
        r.seqMoveWheels(0, 0)
        if r.seqMg.simpleSeqEvent(): r.recalculatePath()
        r.seqResetSequence()
        stMg.changeState("followBest")

    if stMg.checkState("victim"):
        #print("Victim mode!!")
        r.seqMg.startSequence()
        r.seqMoveWheels(0, 0)
        r.seqPrint("stopping")
        r.seqDelaySec(3.2)
        r.seqPrint("reporting")
        if r.seqMg.simpleSeqEvent(): r.reportVictims()
        r.seqPrint("Victim reported")
        r.seqResetSequence()
        stMg.changeState("followBest")

    if stMg.checkState("end"):
        r.seqMg.startSequence()
        if r.seqMg.simpleSeqEvent(): r.endGame()
        r.seqMoveWheels(0, 0)

    #print("--------------------------------------------------------------------")

    #e2 = cv.getTickCount()

    #time = (e2 - e1)/ cv.getTickFrequency()
    #print(f"Tick time is {time}")
    #print(f"Is it optimised? {isOptimised}")

    #print("--------------------------------------------------------------------")