S = {}

local params = {}

function writeWord(h, address, word)
	emu.write_memory(h, address, {word})
end

function readWord(h, address)
	local mem = emu.read_memory(h, address, 1)
	return mem[1]
end

function writeLong(h, address, long)
	emu.write_memory(h, address, {long & 0xFFFF, (long >> 16) & 0xFFFF})
end

function readLong(h, address)
	local mem = emu.read_memory(h, address, 2)
	return mem[1] + (mem[2] << 16)
end

function unblockCsm(h)
	-- read 3 link-pointers (z1 and z2)
	readLong(h, 0x78000)
	readLong(h, 0x78002)
	readLong(h, 0x78004)
	
	readLong(h, 0x78200)
	readLong(h, 0x78202)
	readLong(h, 0x78204)
	
	-- Read SECDC
	readLong(h, 0x703F0)
	
	-- Z1: JLMENABLE
	readLong(h, 0x78006)
	
	-- Z1: PSWDLOCK, CRCLOCK
	readLong(h, 0x78010)
	readLong(h, 0x78012)
	
	-- Z1: GPREG1-4
	readLong(h, 0x78008)
	readLong(h, 0x7800A)
	readLong(h, 0x7800C)
	readLong(h, 0x7800E)
		
	-- Z1: JTAGSWDH0/1
	readLong(h, 0x78014)
	readLong(h, 0x78016)
	
		-- Z2: PSWDLOCK, CRCLOCK
	readLong(h, 0x78210)
	readLong(h, 0x78212)
	
	-- Z2: GPREG1-4
	readLong(h, 0x78208)
	readLong(h, 0x7820A)
	readLong(h, 0x7820C)
	readLong(h, 0x7820E)
	
	local linkPointer = readLong(h, 0x5F000) -- Read Z1-Linkpointer out of Z1-LINKPOINTER register       
	--print("Z1LinkPtr: 0x%X" % {linkPointer})
	   		
	linkPointer = (linkPointer << 18) & 0xFFFFFFFF -- Bits 31 - 14 as most-significant 0 are invalid LinkPointer options
	local bitpos = 13
	local zerofound = 0
	local z1ZoneSelBlockPtr = 0
	while (zerofound == 0) and (bitpos > -1) do
		if (linkPointer & 0x80000000) == 0 then
			zerofound = 1
			z1ZoneSelBlockPtr = (0x78000 + ((bitpos + 2)*0x20))		
		else
			bitpos = bitpos-1
			linkPointer = linkPointer << 1
		end
	end
	if zerofound == 0 then
		z1ZoneSelBlockPtr = 0x78020
	end
	--print("Z1ZoneSelBlockPtr: 0x%X" % {z1ZoneSelBlockPtr})	   		
		
	-- z1 dummy reads	
	for j=4,12 do
		readLong(h, z1ZoneSelBlockPtr + 2*j);
	end
	
	for j=14,15 do
		readLong(h, z1ZoneSelBlockPtr + 2*j);
	end	
	
	for j=0,3 do
		readLong(h, z1ZoneSelBlockPtr + 2*j);
	end	
	
	local linkPointer = readLong(h, 0x5F080) -- Read Z2-Linkpointer out of Z2-LINKPOINTER register       
	--print("Z2LinkPtr: 0x%X" % {linkPointer})
	   		
	linkPointer = (linkPointer << 18) & 0xFFFFFFFF -- Bits 31 - 14 as most-significant 0 are invalid LinkPointer options
	local bitpos = 13
	local zerofound = 0
	local z2ZoneSelBlockPtr = 0
	while (zerofound == 0) and (bitpos > -1) do
		if (linkPointer & 0x80000000) == 0 then
			zerofound = 1
			z2ZoneSelBlockPtr = (0x78200 + ((bitpos + 2)*0x20))		
		else
			bitpos = bitpos-1
			linkPointer = linkPointer << 1
		end
	end
	if zerofound == 0 then
		z2ZoneSelBlockPtr = 0x78220
	end
	--print("Z2ZoneSelBlockPtr: 0x%X" % {z2ZoneSelBlockPtr})	   		
	
	-- z2 dummy reads	
	for j=4,12 do
		readLong(h, z2ZoneSelBlockPtr + 2*j);
	end
		
	for j=0,3 do
		readLong(h, z2ZoneSelBlockPtr + 2*j);
	end	
end

function deviceConfig(h)
    writeLong(h, 0x0005D008, readLong(h, 0x00070212)) --LOAD PARTIDL Value
    writeLong(h, 0x0005D00A, readLong(h, 0x00070214)) --LOAD PARTIDH Value
	
    -- Always enabled
    writeLong(h, 0x0005D012, 0xFFFFFFFF) --LOAD DC1 Value
    writeLong(h, 0x0005D014, 0xFFFFFFFF) --LOAD DC2 Value    
    writeLong(h, 0x0005D016, 0xFFFFFFFF) --LOAD DC3 Value    
    writeLong(h, 0x0005D018, 0xFFFFFFFF) --LOAD DC4 Value    
    writeLong(h, 0x0005D01A, 0xFFFFFFFF) --LOAD DC5 Value
    writeLong(h, 0x0005D01C, 0xFFFFFFFF) --LOAD DC6 Value
    writeLong(h, 0x0005D01E, 0xFFFFFFFF) --LOAD DC7 Value
    writeLong(h, 0x0005D020, 0xFFFFFFFF) --LOAD DC8 Value
    writeLong(h, 0x0005D022, 0xFFFFFFFF) --LOAD DC9 Value
    writeLong(h, 0x0005D024, 0xFFFFFFFF) --LOAD DC10 Value
    writeLong(h, 0x0005D028, 0xFFFFFFFF) --LOAD DC12 Value
    writeLong(h, 0x0005D02A, 0xFFFFFFFF) --LOAD DC13 Value
    writeLong(h, 0x0005D02C, 0xFFFFFFFF) --LOAD DC14 Value
    writeLong(h, 0x0005D02E, 0xFFFFFFFF) --LOAD DC15 Value
    writeLong(h, 0x0005D030, 0xFFFFFFFF) --LOAD DC16 Value
    writeLong(h, 0x0005D032, 0xFFFFFFFF) --LOAD DC17 Value
    writeLong(h, 0x0005D034, 0xFFFFFFFF) --LOAD DC18 Value
    writeLong(h, 0x0005D03C, 0xFFFFFFFF) --LOAD DC22 Value
    writeLong(h, 0x0005D03E, 0xFFFFFFFF) --LOAD DC23 Value
    writeLong(h, 0x0005D040, 0xFFFFFFFF) --LOAD DC24 Value
    writeLong(h, 0x0005D042, 0xFFFFFFFF) --LOAD DC25 Value
    writeLong(h, 0x0005D046, 0xFFFFFFFF) --LOAD DC27 Value
  
    writeLong(h, 0x0005D144, 0xFFFFFFFF) --LOAD CPU1ROM DC3 Value
    writeLong(h, 0x0005D146, 0xFFFFFFFF) --LOAD CPU1ROM DC4 Value
    writeLong(h, 0x0005D154, 0xFFFFFFFF) --LOAD CPU2ROM DC3 Value
    writeLong(h, 0x0005D156, 0xFFFFFFFF) --LOAD CPU2ROM DC4 Value
    
    -- Configured via TI OTP
    writeLong(h, 0x0005D010, (0xFFFF0000 | readWord(h, 0x00070218))) --LOAD DC0 Value
    writeLong(h, 0x0005D026, (0xFFFF0000 | readWord(h, 0x00070219))) --LOAD DC11 Value
    writeLong(h, 0x0005D036, (0xFFFF0000 | readWord(h, 0x0007021A))) --LOAD DC19 Value
    writeLong(h, 0x0005D038, (0xFFFF0000 | readWord(h, 0x0007021B))) --LOAD DC20 Value
    writeLong(h, 0x0005D03A, (0xFFFF0000 | readWord(h, 0x0007021C))) --LOAD DC21 Value
    writeLong(h, 0x0005D044, (0xFFFF0000 | readWord(h, 0x0007021D))) --LOAD DC26 Value    
    writeLong(h, 0x0005D060, (0xFFFF0000 | readWord(h, 0x0007021E))) --LOAD PERCNF1 Value
    writeLong(h, 0x0005D140, (0xFFFF0000 | readWord(h, 0x0007021F))) --LOAD CPU1ROM DC1 Value
    writeLong(h, 0x0005D142, (0xFFFF0000 | readWord(h, 0x00070220))) --LOAD CPU1ROM DC2 Value
    writeLong(h, 0x0005D150, (0xFFFF0000 | readWord(h, 0x00070221))) --LOAD CPU2ROM DC1 Value
    writeLong(h, 0x0005D152, (0xFFFF0000 | readWord(h, 0x00070222))) --LOAD CPU2ROM DC2 Value
    
    -- Lock DCx registers
    writeWord(h, 0x0005D002, readWord(h, 0x0005D002) | 0x1)
end

function aPllConfig(h)
    -- Check APLL key and trim APLL
    if readWord(h, 0x70234) == 0x5A5A then
        -- APLLREFTRIM
        writeWord(h, 0x5D7F0, readWord(h, readLong(h, 0x70240)))
        
        -- SYSAPLLLDOTRIM, SYSAPLLOSCTRIM
        writeWord(h, 0x5D7F1, readWord(h, readLong(h, 0x70240) + 1) & 0xFF)
        writeWord(h, 0x5D7F2, (readWord(h, readLong(h, 0x70240) + 1) & 0x700) >> 0x8)
        
        -- AUXAPLLLDOTRIM, AUXAPLLOSCTRIM
        writeWord(h, 0x5D7F3, readWord(h, readLong(h, 0x70240) + 2) & 0xFF)
        writeWord(h, 0x5D7F4, (readWord(h, readLong(h, 0x70240) + 2) & 0x700) >> 0x8)
    end
    
    -- Configure APLL when key is set
    -- (Analogsubsys SYSAPLCONFIGx/AUXAPLLCONFIGx registers)
     if readWord(h, 0x70237) == 0x5A5A then
        writeWord(h, 0x5D7C0, readWord(h, readLong(h, 0x70246)))
        writeWord(h, 0x5D7D8, readWord(h, readLong(h, 0x70246)))
        
        writeWord(h, 0x5D7C1, readWord(h, readLong(h, 0x70246) + 1))
        writeWord(h, 0x5D7D9, readWord(h, readLong(h, 0x70246) + 1))
        
        writeWord(h, 0x5D7C2, readWord(h, readLong(h, 0x70246) + 2))
        writeWord(h, 0x5D7DA, readWord(h, readLong(h, 0x70246) + 2))
        
        writeWord(h, 0x5D7C3, readWord(h, readLong(h, 0x70246) + 3))
        writeWord(h, 0x5D7DB, readWord(h, readLong(h, 0x70246) + 3))
        
        writeWord(h, 0x5D7C4, readWord(h, readLong(h, 0x70246) + 4))
        writeWord(h, 0x5D7DC, readWord(h, readLong(h, 0x70246) + 4))
        
        writeWord(h, 0x5D7C5, readWord(h, readLong(h, 0x70246) + 5))
        writeWord(h, 0x5D7DD, readWord(h, readLong(h, 0x70246) + 5))
        
        writeWord(h, 0x5D7C6, readWord(h, readLong(h, 0x70246) + 6))
        writeWord(h, 0x5D7DE, readWord(h, readLong(h, 0x70246) + 6))
        
        writeWord(h, 0x5D7C7, readWord(h, readLong(h, 0x70246) + 7))
        writeWord(h, 0x5D7DF, readWord(h, readLong(h, 0x70246) + 7))
        
        writeWord(h, 0x5D7C8, readWord(h, readLong(h, 0x70246) + 8))
        writeWord(h, 0x5D7E0, readWord(h, readLong(h, 0x70246) + 8))
        
        writeWord(h, 0x5D7C9, readWord(h, readLong(h, 0x70246) + 9))
        writeWord(h, 0x5D7E1, readWord(h, readLong(h, 0x70246) + 9))
	end
end

function S.setParameter(h, k, v)
   params[k] = v
end

function S.onConnect_D(h)
	if(params['mode'] == 'nonintrusive') then
		return
	end
end	

function S.onReset(h)
	if(params['mode'] == 'nonintrusive') then
		return
	end

    if(params['bootmode'] == 'flash') then
        writeLong(h, 0xD00, 0x5AFFFFFF) 
        writeLong(h, 0xD04, 0x0003) 
    elseif (params['bootmode'] == 'ram') then
    	--print('Boot from RAM...')
    	
    	--writeWord(h, 0x5FB00, 0) -- disable Flash ECC
    	writeWord(h, 0x7029, 0x68) -- disable WD	

	    -- (re) secure both zones
		writeLong(h, 0x0005F018, 0x80000000)
		writeLong(h, 0x0005F098, 0x80000000)
     	unblockCsm(h)
        
        -- reset slave cores
        writeLong(h, 0x0005DC00, 0xA5A50001) -- CM
        writeLong(h, 0x0005D122, 0xA5A50001) -- CPU2

	    -- initalize memories
	    writeWord(h, 0x5F412, 0x000F) -- RAM INIT FOR M0/M1/D0/D1 Memory
	    writeWord(h, 0x5F432, 0x003F) -- RAM INIT FOR LS0..LS5  Memory
	    writeWord(h, 0x5F452, 0x0FFF) -- RAM INIT FOR GS0..GS11 
        
        -- configure boot from SRAM
   		writeLong(h, 0xD00, 0x5AFFFFFF) 
    	writeLong(h, 0xD04, 0x0005)
    else    
	    -- "custom" bootmode
	 	-- writeWord(h, 0x5FB00, 0) -- disable Flash ECC
		writeWord(h, 0x7029, 0x68) -- disable WD	
		
	    -- (re) secure both zones
		writeLong(h, 0x0005F018, 0x80000000)
		writeLong(h, 0x0005F098, 0x80000000)
				
		-- InitDCSM()
		unblockCsm(h)
				
		deviceConfig(h)
		aPllConfig(h)
		
		--CLA_Clock_Enable();
	    --ERAD_Clock_Enable();
	    
	    -- If CPU2 isn't out of reset, boot CPU2 to wait boot
	    if readLong(h, 0x0005D122) == 0x1 then
	        writeLong(h, 0x5CE22, 0x5A00D200) -- CPU1TOCPU2IPCBOOTMODE 
	    	writeLong(h, 0x5CE04, 0x1) -- Set flag0 of CPU1TOCPU2IPCSET 
	    	writeLong(h, 0x0005D122, 0xA5A50000) -- Bring CPU2 out of reset.
	    	print("CPU2 is out of reset and configured to wait boot.")           
	    end
	    
	    -- If CM isn't out of reset, boot CM to wait boot
	    if readLong(h, 0x0005DC00) == 0x1 then
	    	writeLong(h, 0x5CE62, 0x5A007D00) -- CPU1TOCMIPCBOOTMODE
	    	writeLong(h, 0x5CE44, 0x1) -- Set flag0 of CPU1TOCMIPCSET 
	    	writeLong(h, 0x0005DC00, 0xA5A50000) -- Bring CM out of reset.
	      	print("CM is out of reset and configured to wait boot.");       
	    end
	 				
	    -- OnTargetConnect()
	    writeWord(h, 0x5F412, 0x000F) -- RAM INIT FOR M0/M1/D0/D1 Memory
	    writeWord(h, 0x5F432, 0x003F) -- RAM INIT FOR LS0..LS5  Memory
	    writeWord(h, 0x5F452, 0x0FFF) -- RAM INIT FOR GS0..GS11 Memory
	end 
 end

function S.onDisconnect(h)
	if(params['mode'] == 'nonintrusive') then
		return
	end
	
   emu.halt(h)
   emu.reset(h)   
   emu.runFree(h) 
end

return S