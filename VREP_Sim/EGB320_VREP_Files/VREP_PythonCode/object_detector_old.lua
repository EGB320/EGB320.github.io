simVision = require('simVision')
sim = require('sim')

function sysCall_init()
    -- do some initialization here
end

function sysCall_actuation()
    -- put your actuation code here
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

function sysCall_vision(inData)
    -- callback function automatically added for backward compatibility
    -- (vision sensor have no filters anymore, but rather a callback function where image processing can be performed)
    
    -- Start timing
    local startTime = sim.getSystemTime()
    
    --returns detection array for: bowl,mug,bottle,soccer ball,rubiks cube,cereal box,obstacle_0,obstacle_1,obstacle_2,packing_bay,row marker1,row marker2,row marker3,shelf0-5,square_marker1-3,picking_station
    unpackedPackets={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
    
    -- Copy sensor image to work image and save to buffer1 for restoration
    simVision.sensorImgToWorkImg(inData.handle)
    simVision.workImgToBuffer1(inData.handle)  -- Save original image to buffer1
    
    ----
    --This code below checks for aux component on each different object and performs blob detection and checks if a blob is found
    ----
    
    -- Warehouse items with correct aux channel values
    -- Bowl = 1.00, 0.48, 0.0
    simVision.buffer1ToWorkImg(inData.handle)
    simVision.selectiveColorOnWorkImg(inData.handle,{1.00,0.48,0.00},{0.05,0.05,0.05},true,true,false)
    local trig,packedPacket_item_bowl=simVision.blobDetectionOnWorkImg(inData.handle,0.150,0.001,false, {0.0,1.0,0.0}) 
    if packedPacket_item_bowl then
        t_sample_0 = sim.unpackFloatTable(packedPacket_item_bowl, 0,0,0)
        if t_sample_0[1] > 0 then
            unpackedPackets[1] = 1
            print("BOWL detected! Blob count:", t_sample_0[1])
        end
    end
    
    -- Mug = 1.00, 0.00, 0.60
    simVision.buffer1ToWorkImg(inData.handle)
    simVision.selectiveColorOnWorkImg(inData.handle,{1.00,0.00,0.60},{0.05,0.05,0.05},true,true,false)
    local trig,packedPacket_item_mug=simVision.blobDetectionOnWorkImg(inData.handle,0.150,0.001,false, {0.0,1.0,0.0}) 
    if packedPacket_item_mug then
        t_sample_1 = sim.unpackFloatTable(packedPacket_item_mug, 0,0,0)
        if t_sample_1[1] > 0 then
            unpackedPackets[2] = 1
            print("MUG detected! Blob count:", t_sample_1[1])
        end
    end
    
    -- Bottle = 1.00, 0.00, 0.00
    simVision.buffer1ToWorkImg(inData.handle)
    simVision.selectiveColorOnWorkImg(inData.handle,{1.00,0.00,0.00},{0.05,0.05,0.05},true,true,false)
    local trig,packedPacket_item_bottle=simVision.blobDetectionOnWorkImg(inData.handle,0.150,0.001,false, {0.0,1.0,0.0}) 
    if packedPacket_item_bottle then
        t_sample_2 = sim.unpackFloatTable(packedPacket_item_bottle, 0,0,0)
        if t_sample_2[1] > 0 then
            unpackedPackets[3] = 1
            print("BOTTLE detected! Blob count:", t_sample_2[1])
        end
    end
    
    -- Ball = 0.00, 0.58, 1.0
    simVision.buffer1ToWorkImg(inData.handle)
    simVision.selectiveColorOnWorkImg(inData.handle,{0.00,0.58,1.00},{0.05,0.05,0.05},true,true,false)
    local trig,packedPacket_item_soccer_ball=simVision.blobDetectionOnWorkImg(inData.handle,0.150,0.001,false, {0.0,1.0,0.0}) 
    if packedPacket_item_soccer_ball then
        t_rock_0 = sim.unpackFloatTable(packedPacket_item_soccer_ball, 0,0,0)
        if t_rock_0[1] > 0 then
            unpackedPackets[4] = 1
            print("BALL detected! Blob count:", t_rock_0[1])
        end
    end
    
    -- Rubiks Cube = 0.02, 0.0, 1.0
    simVision.buffer1ToWorkImg(inData.handle)
    simVision.selectiveColorOnWorkImg(inData.handle,{0.02,0.00,1.00},{0.05,0.05,0.05},true,true,false)
    local trig,packedPacket_item_rubiks_cube=simVision.blobDetectionOnWorkImg(inData.handle,0.150,0.001,false, {0.0,1.0,0.0}) 
    if packedPacket_item_rubiks_cube then
        t_rock_1 = sim.unpackFloatTable(packedPacket_item_rubiks_cube, 0,0,0)
        if t_rock_1[1] > 0 then
            unpackedPackets[5] = 1
            print("RUBIKS CUBE detected! Blob count:", t_rock_1[1])
        end
    end
    
    -- Cereal = 0.62, 0.0, 1.0
    simVision.buffer1ToWorkImg(inData.handle)
    simVision.selectiveColorOnWorkImg(inData.handle,{0.62,0.00,1.00},{0.05,0.05,0.05},true,true,false)
    local trig,packedPacket_item_cereal_box=simVision.blobDetectionOnWorkImg(inData.handle,0.150,0.001,false, {0.0,1.0,0.0}) 
    if packedPacket_item_cereal_box then
        t_rock_2 = sim.unpackFloatTable(packedPacket_item_cereal_box, 0,0,0)
        if t_rock_2[1] > 0 then
            unpackedPackets[6] = 1
            print("CEREAL detected! Blob count:", t_rock_2[1])
        end
    end
    
    -- Obstacles with correct aux channel values
    -- Obstacle_0 = 0.02, 1.00, 0.00
    simVision.buffer1ToWorkImg(inData.handle)
    simVision.selectiveColorOnWorkImg(inData.handle,{0.02,1.00,0.00},{0.05,0.05,0.05},true,true,false)
    local trig,packedPacket_obstacle_0=simVision.blobDetectionOnWorkImg(inData.handle,0.150,0.001,false, {0.0,0.0,1.0})
    if packedPacket_obstacle_0 then
        t_obstacle_0 = sim.unpackFloatTable(packedPacket_obstacle_0, 0,0,0)
        if t_obstacle_0[1] > 0 then
            unpackedPackets[7] = 1
            print("OBSTACLE_0 detected! Blob count:", t_obstacle_0[1])
        end
    end
    
    -- Obstacle_1 = 0.15, 1.00, 0.15
    simVision.buffer1ToWorkImg(inData.handle)
    simVision.selectiveColorOnWorkImg(inData.handle,{0.15,1.00,0.15},{0.05,0.05,0.05},true,true,false)
    local trig,packedPacket_obstacle_1=simVision.blobDetectionOnWorkImg(inData.handle,0.150,0.001,false, {0.0,0.0,1.0}) 
    if packedPacket_obstacle_1 then
        t_obstacle_1 = sim.unpackFloatTable(packedPacket_obstacle_1, 0,0,0)
        if t_obstacle_1[1] > 0 then
            unpackedPackets[8] = 1
            print("OBSTACLE_1 detected! Blob count:", t_obstacle_1[1])
        end
    end
    
    -- Obstacle_2 = 0.00, 0.5, 0.29
    simVision.buffer1ToWorkImg(inData.handle)
    simVision.selectiveColorOnWorkImg(inData.handle,{0.00,0.50,0.29},{0.05,0.05,0.05},true,true,false)
    local trig,packedPacket_obstacle_2=simVision.blobDetectionOnWorkImg(inData.handle,0.150,0.001,false, {0.0,0.0,1.0}) 
    if packedPacket_obstacle_2 then
        t_obstacle_2 = sim.unpackFloatTable(packedPacket_obstacle_2, 0,0,0)
        if t_obstacle_2[1] > 0 then
            unpackedPackets[9] = 1
            print("OBSTACLE_2 detected! Blob count:", t_obstacle_2[1])
        end
    end
    
    -- PACKING BAY (keeping existing detection logic)
    simVision.buffer1ToWorkImg(inData.handle)
    simVision.selectiveColorOnWorkImg(inData.handle,{0.17,1.00,0.5},{0.04,0.50,1.0},true,true,false)
    local trig,packedPacket_packing_bay=simVision.blobDetectionOnWorkImg(inData.handle,0.150,0.001,false, {0.0,0.0,1.0}) 
    if packedPacket_packing_bay then
        t_lander = sim.unpackFloatTable(packedPacket_packing_bay, 0,0,0)
        if t_lander[1] > 0 then
            unpackedPackets[10] = 1
            print("PACKING BAY detected! Blob count:", t_lander[1])
        end
    end
    
    -- ROW MARKERS with well-separated aux channel values
    -- row_marker1 = 0.15, 0.10, 0.05
    simVision.buffer1ToWorkImg(inData.handle)
    simVision.selectiveColorOnWorkImg(inData.handle,{0.15,0.10,0.05},{0.02,0.02,0.02},true,true,false)
    local trig,packedPacket_row_marker1=simVision.blobDetectionOnWorkImg(inData.handle,0.001,0.001,false, {0.0,0.0,1.0}) 
    if packedPacket_row_marker1 then
        t_marker_1 = sim.unpackFloatTable(packedPacket_row_marker1, 0,0,0)
        if t_marker_1[1] > 0 then
            unpackedPackets[11] = 1
            print("ROW_MARKER_1 detected! Blob count:", t_marker_1[1])
        end
    end
    
    -- row_marker2 = 0.10, 0.15, 0.05
    simVision.buffer1ToWorkImg(inData.handle)
    simVision.selectiveColorOnWorkImg(inData.handle,{0.10,0.15,0.05},{0.02,0.02,0.02},true,true,false)
    local trig,packedPacket_row_marker2=simVision.blobDetectionOnWorkImg(inData.handle,0.001,0.001,false, {0.0,0.0,1.0}) 
    if packedPacket_row_marker2 then
        t_marker_2 = sim.unpackFloatTable(packedPacket_row_marker2, 0,0,0)
        if t_marker_2[1] > 0 then
            unpackedPackets[12] = 1
            print("ROW_MARKER_2 detected! Blob count:", t_marker_2[1])
        end
    end
    
    -- row_marker3 = 0.05, 0.10, 0.15
    simVision.buffer1ToWorkImg(inData.handle)
    simVision.selectiveColorOnWorkImg(inData.handle,{0.05,0.10,0.15},{0.02,0.02,0.02},true,true,false)
    local trig,packedPacket_row_marker3=simVision.blobDetectionOnWorkImg(inData.handle,0.001,0.001,false, {0.0,0.0,1.0}) 
    if packedPacket_row_marker3 then
        t_marker_3 = sim.unpackFloatTable(packedPacket_row_marker3, 0,0,0)
        if t_marker_3[1] > 0 then
            unpackedPackets[13] = 1
            print("ROW_MARKER_3 detected! Blob count:", t_marker_3[1])
        end
    end
    
    -- Shelves with correct aux channel values
    -- shelf0 = 0.33, 0.306, 0.27
    simVision.buffer1ToWorkImg(inData.handle)
    simVision.selectiveColorOnWorkImg(inData.handle,{0.33,0.306,0.27},{0.02,0.02,0.02},true,true,false)
    local trig,packedPacket_shelf_0=simVision.blobDetectionOnWorkImg(inData.handle,0.001,0.001,false, {0.0,0.0,1.0}) 
    if packedPacket_shelf_0 then
        t_shelf_0 = sim.unpackFloatTable(packedPacket_shelf_0, 0,0,0)
        if t_shelf_0[1] > 0 then
            unpackedPackets[14] = 1
            print("SHELF_0 detected! Blob count:", t_shelf_0[1])
        end
    end
    
    -- shelf1 = 0.318, 0.33, 0.27
    simVision.buffer1ToWorkImg(inData.handle)
    simVision.selectiveColorOnWorkImg(inData.handle,{0.318,0.33,0.27},{0.02,0.02,0.02},true,true,false)
    local trig,packedPacket_shelf_1=simVision.blobDetectionOnWorkImg(inData.handle,0.001,0.001,false, {0.0,0.0,1.0}) 
    if packedPacket_shelf_1 then
        t_shelf_1 = sim.unpackFloatTable(packedPacket_shelf_1, 0,0,0)
        if t_shelf_1[1] > 0 then
            unpackedPackets[15] = 1
            print("SHELF_1 detected! Blob count:", t_shelf_1[1])
        end
    end
    
    -- shelf2 = 0.282, 0.33, 0.27
    simVision.buffer1ToWorkImg(inData.handle)
    simVision.selectiveColorOnWorkImg(inData.handle,{0.282,0.33,0.27},{0.02,0.02,0.02},true,true,false)
    local trig,packedPacket_shelf_2=simVision.blobDetectionOnWorkImg(inData.handle,0.001,0.001,false, {0.0,0.0,1.0}) 
    if packedPacket_shelf_2 then
        t_shelf_2 = sim.unpackFloatTable(packedPacket_shelf_2, 0,0,0)
        if t_shelf_2[1] > 0 then
            unpackedPackets[16] = 1
            print("SHELF_2 detected! Blob count:", t_shelf_2[1])
        end
    end
    
    -- shelf3 = 0.27, 0.33, 0.294
    simVision.buffer1ToWorkImg(inData.handle)
    simVision.selectiveColorOnWorkImg(inData.handle,{0.27,0.33,0.294},{0.02,0.02,0.02},true,true,false)
    local trig,packedPacket_shelf_3=simVision.blobDetectionOnWorkImg(inData.handle,0.001,0.001,false, {0.0,0.0,1.0}) 
    if packedPacket_shelf_3 then
        t_shelf_3 = sim.unpackFloatTable(packedPacket_shelf_3, 0,0,0)
        if t_shelf_3[1] > 0 then
            unpackedPackets[17] = 1
            print("SHELF_3 detected! Blob count:", t_shelf_3[1])
        end
    end
    
    -- shelf4 = 0.27, 0.33, 0.33
    simVision.buffer1ToWorkImg(inData.handle)
    simVision.selectiveColorOnWorkImg(inData.handle,{0.27,0.33,0.33},{0.02,0.02,0.02},true,true,false)
    local trig,packedPacket_shelf_4=simVision.blobDetectionOnWorkImg(inData.handle,0.001,0.001,false, {0.0,0.0,1.0}) 
    if packedPacket_shelf_4 then
        t_shelf_4 = sim.unpackFloatTable(packedPacket_shelf_4, 0,0,0)
        if t_shelf_4[1] > 0 then
            unpackedPackets[18] = 1
            print("SHELF_4 detected! Blob count:", t_shelf_4[1])
        end
    end
    
    -- shelf5 = 0.27, 0.294, 0.33
    simVision.buffer1ToWorkImg(inData.handle)
    simVision.selectiveColorOnWorkImg(inData.handle,{0.27,0.294,0.33},{0.02,0.02,0.02},true,true,false)
    local trig,packedPacket_shelf_5=simVision.blobDetectionOnWorkImg(inData.handle,0.001,0.001,false, {0.0,0.0,1.0}) 
    if packedPacket_shelf_5 then
        t_shelf_5 = sim.unpackFloatTable(packedPacket_shelf_5, 0,0,0)
        if t_shelf_5[1] > 0 then
            unpackedPackets[19] = 1
            print("SHELF_5 detected! Blob count:", t_shelf_5[1])
        end
    end
    
    -- Square markers
    -- square_marker_1 = 0.75, 0.25, 0.50
    simVision.buffer1ToWorkImg(inData.handle)
    simVision.selectiveColorOnWorkImg(inData.handle,{0.75,0.25,0.50},{0.05,0.05,0.05},true,true,false)
    local trig,packedPacket_square_marker_1=simVision.blobDetectionOnWorkImg(inData.handle,0.001,0.001,false, {1.0,0.0,1.0}) 
    if packedPacket_square_marker_1 then
        t_square_marker_1 = sim.unpackFloatTable(packedPacket_square_marker_1, 0,0,0)
        if t_square_marker_1[1] > 0 then
            unpackedPackets[20] = 1
            print("SQUARE_MARKER_1 detected! Blob count:", t_square_marker_1[1])
        end
    end
    
    -- square_marker_2 = 0.50, 0.75, 0.25
    simVision.buffer1ToWorkImg(inData.handle)
    simVision.selectiveColorOnWorkImg(inData.handle,{0.50,0.75,0.25},{0.05,0.05,0.05},true,true,false)
    local trig,packedPacket_square_marker_2=simVision.blobDetectionOnWorkImg(inData.handle,0.001,0.001,false, {1.0,0.0,1.0}) 
    if packedPacket_square_marker_2 then
        t_square_marker_2 = sim.unpackFloatTable(packedPacket_square_marker_2, 0,0,0)
        if t_square_marker_2[1] > 0 then
            unpackedPackets[21] = 1
            print("SQUARE_MARKER_2 detected! Blob count:", t_square_marker_2[1])
        end
    end
    
    -- square_marker_3 = 0.25, 0.50, 0.75
    simVision.buffer1ToWorkImg(inData.handle)
    simVision.selectiveColorOnWorkImg(inData.handle,{0.25,0.50,0.75},{0.05,0.05,0.05},true,true,false)
    local trig,packedPacket_square_marker_3=simVision.blobDetectionOnWorkImg(inData.handle,0.001,0.001,false, {1.0,0.0,1.0}) 
    if packedPacket_square_marker_3 then
        t_square_marker_3 = sim.unpackFloatTable(packedPacket_square_marker_3, 0,0,0)
        if t_square_marker_3[1] > 0 then
            unpackedPackets[22] = 1
            print("SQUARE_MARKER_3 detected! Blob count:", t_square_marker_3[1])
        end
    end
    
    -- Picking station main structure
    -- picking_station = 0.90, 0.45, 0.20
    simVision.buffer1ToWorkImg(inData.handle)
    simVision.selectiveColorOnWorkImg(inData.handle,{0.90,0.45,0.20},{0.05,0.05,0.05},true,true,false)
    local trig,packedPacket_picking_station=simVision.blobDetectionOnWorkImg(inData.handle,0.150,0.001,false, {1.0,0.5,0.0}) 
    if packedPacket_picking_station then
        t_picking_station = sim.unpackFloatTable(packedPacket_picking_station, 0,0,0)
        if t_picking_station[1] > 0 then
            unpackedPackets[23] = 1
            print("PICKING_STATION detected! Blob count:", t_picking_station[1])
        end
    end
    

    simVision.buffer1ToWorkImg(inData.handle)
    
    -- Calculate and print processing time
    local endTime = sim.getSystemTime()
    local processingTime = (endTime - startTime) * 1000  -- Convert to milliseconds
    print(string.format("Fixed processing time: %.2f ms", processingTime))
    
    --print(unpackedPackets)

    outData={}
    outData.trigger=false -- whether the sensor should trigger
    outData.packedPackets={sim.packFloatTable(unpackedPackets)}
    --outData.packedPackets={sim.packFloatTable(unpackedPackets)} -- filters may append packets (in packed form, use sim.packFloatTable to pack) to this table
    return outData
    
end

-- See the user manual or the available code snippets for additional callback functions and details
