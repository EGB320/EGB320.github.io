function __setObjectPosition__(a,b,c)
    -- compatibility routine, wrong results could be returned in some situations, in CoppeliaSim <4.0.1
    if b==sim.handle_parent then
        b=sim.getObjectParent(a)
    end
    if (b~=-1) and (sim.getObjectType(b)==sim.object_joint_type) and (sim.getInt32Param(sim.intparam_program_version)>=40001) then
        a=a+sim.handleflag_reljointbaseframe
    end
    return sim.setObjectPosition(a,b,c)
end
function __getObjectPosition__(a,b)
    -- compatibility routine, wrong results could be returned in some situations, in CoppeliaSim <4.0.1
    if b==sim.handle_parent then
        b=sim.getObjectParent(a)
    end
    if (b~=-1) and (sim.getObjectType(b)==sim.object_joint_type) and (sim.getInt32Param(sim.intparam_program_version)>=40001) then
        a=a+sim.handleflag_reljointbaseframe
    end
    return sim.getObjectPosition(a,b)
end
function sysCall_init()
    -- do some initialization here:
    simRemoteApi.start(19999)

    -- ensure initial motor velocities are zero
    leftMotor = sim.getObjectHandle("LeftMotor")
    rightMotor = sim.getObjectHandle("RightMotor")
    sim.setJointTargetVelocity(leftMotor,0)
    sim.setJointTargetVelocity(rightMotor,0)
    
    robotHandle = sim.getObjectHandle("Robot")
    
    dummy0Handle = sim.getObjectHandle("Dummy0")
    dummy2Handle = sim.getObjectHandle("Dummy2")
    dummy4Handle = sim.getObjectHandle("Dummy4")
    
    dummyCollectorHandles = {dummy0Handle, dummy2Handle, dummy4Handle}
    
    dummy1Handle = sim.getObjectHandle("Dummy1")
    dummy3Handle = sim.getObjectHandle("Dummy3")
    dummy5Handle = sim.getObjectHandle("Dummy5")
    dummySampleHandles = {dummy1Handle, dummy3Handle, dummy5Handle}
    
    obstacle0Handle = sim.getObjectHandle("Obstacle_0")
    obstacle1Handle = sim.getObjectHandle("Obstacle_1")
    obstacle2Handle = sim.getObjectHandle("Obstacle_2")
    
    obstacleHandles = {obstacle0Handle,obstacle1Handle,obstacle2Handle}
    
    object_detector = sim.getObjectHandle("ObjectDetector")   
    
    itemTemplateHandles = {
        sim.getObject("/HoldingTable/BOWL"),
        sim.getObject("/HoldingTable/MUG"),
        sim.getObject("/HoldingTable/BOTTLE"),
        sim.getObject("/HoldingTable/SOCCER_BALL"),
        sim.getObject("/HoldingTable/RUBIKS_CUBE"),
        sim.getObject("/HoldingTable/CEREAL_BOX"),
    }
    itemNamesLookup = {
        "BOWL",
        "MUG",
        "BOTTLE",
        "SOCCER_BALL",
        "RUBIKS_CUBE",
        "CEREAL_BOX"
    }
    
    warehouseItemHandles = {}
    heldItems = {}


    -- Make sure you read the section on "Accessing general-type objects programmatically"
    -- For instance, if you wish to retrieve the handle of a scene object, use following instruction:
    --
    -- handle=sim.getObjectHandle('sceneObjectName')
    -- 
    -- Above instruction retrieves the handle of 'sceneObjectName' if this script's name has no '#' in it
    --
    -- If this script's name contains a '#' (e.g. 'someName#4'), then above instruction retrieves the handle of object 'sceneObjectName#4'
    -- This mechanism of handle retrieval is very convenient, since you don't need to adjust any code when a model is duplicated!
    -- So if the script's name (or rather the name of the object associated with this script) is:
    --
    -- 'someName', then the handle of 'sceneObjectName' is retrieved
    -- 'someName#0', then the handle of 'sceneObjectName#0' is retrieved
    -- 'someName#1', then the handle of 'sceneObjectName#1' is retrieved
    -- ...
    --
    -- If you always want to retrieve the same object's handle, no matter what, specify its full name, including a '#':
    --
    -- handle=sim.getObjectHandle('sceneObjectName#') always retrieves the handle of object 'sceneObjectName' 
    -- handle=sim.getObjectHandle('sceneObjectName#0') always retrieves the handle of object 'sceneObjectName#0' 
    -- handle=sim.getObjectHandle('sceneObjectName#1') always retrieves the handle of object 'sceneObjectName#1'
    -- ...
    --
    -- Refer also to sim.getCollisionhandle, sim.getDistanceHandle, sim.getIkGroupHandle, etc.
end

function sysCall_actuation()
    -- put your actuation code here

    --local trigger,packet1,packet2,packet3=sim.handleVisionSensor(object_detector)
    --sim.handleVisionSensor(object_detector) -- the image processing camera is handled explicitely, since we do not need to execute that command at each simulation pass
    --result,detected_objects=sim.readVisionSensor(object_detector) 
    

    
    --
    -- For example:
    --
    -- local position=sim.getObjectPosition(handle,-1)
    -- position[1]=position[1]+0.001
    -- sim.setObjectPosition(handle,-1,position)
end



function sysCall_cleanup()
    -- do some clean-up here
end

getDistanceToObject=function(inInts,inFloats,inStrings,inBuffer)
    results = {}
    distanceData = {}
    for i=1,#inInts do
        threshold = inFloats[i]
        result, data, handles = sim.checkDistance(robotHandle,inInts[i],threshold)
        
        -- make it consistent with other error codes where 0 == success.
        if result == 1 then 
            result = 0
        else
            result = 1
        end
        table.insert(results,result)
        for i=1,#data,1 do
            distanceData[#distanceData+1] = data[i]
        end
        
    end
    return results,distanceData,{},''

end

getObjectsInView=function(inInts,inFloats,inStrings,inBuffer)

    local trigger,packet1,packet2=sim.handleVisionSensor(object_detector)
    --sim.handleVisionSensor(object_detector) -- the image processing camera is handled explicitely, since we do not need to execute that command at each simulation pass
    --result,detected_objects=sim.readVisionSensor(object_detector) 
    --print("Packet 2 contains: ",packet2)

    return packet2,{},{},''

end

SetBayContents=function(inInts,inFloats,inStrings,inBuffer)

    shelf = inInts[1]
    bayX = inInts[2]
    bayY = inInts[3]
    itemTypeEnum = inInts[4]
    
    bayHandle = sim.getObject("/Shelf" .. shelf .. "/Bay" .. bayX .. bayY)
    itemName = itemNamesLookup[itemTypeEnum] .. shelf .. bayX .. bayY
    itemHandle = sim.getObject("/" .. itemName,{noError=true})
    if itemHandle == -1 then
        templateHandle = itemTemplateHandles[itemTypeEnum]
        -- Make 1 clone with 0 edits.
        itemHandle = sim.copyPasteObjects({templateHandle},0)[1]
        sim.setObjectAlias(itemHandle,itemName)
    end
    __setObjectPosition__(itemHandle,bayHandle,{0,0,0})
    table.insert(warehouseItemHandles,itemHandle)
end

JoinRobotAndItem = function(inInts, inFloats, inStrings,inBuffer)
    print("grab")
    for i=1,#inInts,1 do
        local objectHandle = inInts[i]
        __setObjectPosition__(objectHandle,dummySampleHandles[1],{0,0,0.03})
        sim.setObjectParent(objectHandle,dummySampleHandles[1],true)
        table.insert(heldItems,objectHandle)
        print("Grabbed Item: ")
        print(objectHandle)
    end
end


RobotReleaseItem=function(inInts,inFLoats,inStrings,inBuffer)
    if #heldItems > 0 then
        objectHandle = table.remove(heldItems,1)
        sim.setObjectParent(objectHandle,-1,true)
        sim.setObjectInt32Param(objectHandle,3004,1) -- make object respondable
        sim.setObjectInt32Param(objectHandle,sim.shapeintparam_static,0)
    end
end

--JoinRobotAndItem=function(inInts, inFloats, inStrings, inBuffer)
--    if #inInts >= 1 and #inFloats >= 1 then
--        local collect = inInts[1]
--        local max_distance = inFloats[1]
--        if collect == 1 then
--           
--            -- make ball parent of dummy1
--            for i=1,#sampleHandles,1 do             
--               sim.handleDistance(distanceHandles[i])
--               local result, distance1 = sim.readDistance(distanceHandles[i])
--
--               if distance1 <= 1.5*max_distance then
--                    local position = {0,0.01,0.03}
--                    __setObjectPosition__(sampleHandles[i], dummySampleHandles[i],position)
--                    sim.setObjectParent(sampleHandles[i], dummySampleHandles[i],  true)
--                    print("Grabbing Item Number:")
--                    print(i)
--                end
--            end
--        else
--            -- make dummy0 parent of dummy1
--            print("Dropping Item")
--            sim.setObjectParent(dummySampleHandles[1], dummyCollectorHandles[1], true)
--            sim.setObjectParent(dummySampleHandles[2], dummyCollectorHandles[2], true)
--            sim.setObjectParent(dummySampleHandles[3], dummyCollectorHandles[3], true)
--        end
--    end
--
--    return {},{},{},''
--end

-- You can define additional system calls here:
--[[
function sysCall_suspend()
end

function sysCall_resume()
end

function sysCall_dynCallback(inData)
end

function sysCall_jointCallback(inData)
    return outData
end

function sysCall_contactCallback(inData)
    return outData
end

function sysCall_beforeCopy(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." will be copied")
    end
end

function sysCall_afterCopy(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." was copied")
    end
end

function sysCall_beforeDelete(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." will be deleted")
    end
    -- inData.allObjects indicates if all objects in the scene will be deleted
end

function sysCall_afterDelete(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." was deleted")
    end
    -- inData.allObjects indicates if all objects in the scene were deleted
end
--]]
