-- emulator script

local S = {}

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

function S.setParameter(h, k, v)
   params[k] = v
end

function S.onReset(h)
    if(params['bootmode'] == 'flash') then
        writeLong(h, 0xD00, 0x5AFFFFFF) -- EMU_BOOTPIN_CONFIG (GPREG1) = 0x5AFF FFFF
        writeLong(h, 0xD04, 0x00000003) -- EMU_BOOTDEF_LOW (GPREG3)    = 0x0000 0003
        return
    end
    
    -- "custom" bootmode:
    
	--print("Emulator reset.")
	writeWord(0x5FB00, 0) -- disable Flash ECC
	writeWord(0x7029, 0x68) -- disable WD	
	
	--[[
	   *(unsigned long *)0x7060 = 0x0001 -- Enable NMI
	--]]
	
	readLong(h, 0x78000) -- Read BANK0 Z1 Linkpointer1
	readLong(h, 0x78004) -- Read BANK0 Z1 Linkpointer2
	readLong(h, 0x78008) -- Read BANK0 Z1 Linkpointer3 
	 
	readLong(h, 0x78200) -- Read BANK0 Z2 Linkpointer1
	readLong(h, 0x78204) -- Read BANK0 Z2 Linkpointer2
	readLong(h, 0x78208) -- Read BANK0 Z2 Linkpointer3

	readLong(h, 0x78400) -- Read BANK1 Z1 Linkpointer1
	readLong(h, 0x78404) -- Read BANK1 Z1 Linkpointer2
	readLong(h, 0x78408) -- Read BANK1 Z1 Linkpointer3 
	 
	readLong(h, 0x78600) -- Read BANK1 Z2 Linkpointer1
	readLong(h, 0x78604) -- Read BANK1 Z2 Linkpointer2
	readLong(h, 0x78608) -- Read BANK1 Z2 Linkpointer3
	  
	readLong(h, 0x703F0) -- Read SECDC   
	   
	readLong(h, 0x78010) -- Read Z1 PSWDLOCK
	readLong(h, 0x78014) -- Read Z1 CRCLOCK
	readLong(h, 0x78018) -- Read Z1 JTAGLOCK
	readLong(h, 0x7801E) -- Read Z1 BOOTMODE

	readLong(h, 0x7800C) -- Read Z1 GPREG1
	readLong(h, 0x7800E) -- Read Z1 GPREG2
	readLong(h, 0x7801C) -- Read Z1 GPREG3

	readLong(h, 0x78210) -- Read Z2 PSWDLOCK
	readLong(h, 0x78214) -- Read Z2 CRCLOCK
	readLong(h, 0x78218) -- Read Z2 JTAGLOCK
	readLong(h, 0x7821E) -- Read Z2 BOOTMODE

	readLong(h, 0x7820C) -- Read Z2 GPREG1
	readLong(h, 0x7820E) -- Read Z2 GPREG2
	readLong(h, 0x7821C) -- Read Z2 GPREG3
	   	
	for i=0,1 do
	    local linkPointer = readLong(h, 0x5F000 + (i * 256)) -- Read Z1-Linkpointer out of Z1-LINKPOINTER register       
	    --print("Z1LinkPtr: 0x%X" % {linkPointer})
	   		
	   	linkPointer = (linkPointer << 3) & 0xFFFFFFFF -- Bits 31,30 and 29 as most-significant 0 are invalid LinkPointer options
		local bitpos = 28
		local zerofound = 0
		local z1ZoneSelBlockPtr = 0
		while (zerofound == 0) and (bitpos > -1) do
	        if (linkPointer & 0x80000000) == 0 then
		        zerofound = 1
				z1ZoneSelBlockPtr = (0x78000 + ((bitpos + 3)*16 + (i * 1024)))		
			else
				bitpos = bitpos-1
			    linkPointer = linkPointer << 1
			end
		end
		if zerofound == 0 then
			z1ZoneSelBlockPtr = 0x78020 + (i * 1024)
		end
		--print("Z1ZoneSelBlockPtr: 0x%X" % {z1ZoneSelBlockPtr})
		
		--[[
			Perform dummy reads of the Zone Select Block locations
			0: Zx-EXEONLYRAM
			2: Zx-EXEONLYSECT
			4: Zx-GRABRAM
			6: Zx-GRABSECT
			8: Zx-CSMPSWD0
			A: Zx-CSMPSWD0
			B: Zx-CSMPSWD0
			C: Zx-CSMPSWD0
		--]]
		for j=0,8 do
			readLong(h, z1ZoneSelBlockPtr);
			z1ZoneSelBlockPtr = z1ZoneSelBlockPtr + 2
		end
	end
	
	for i=0,1 do
	    local linkPointer = readLong(h, 0x5F040 + (i * 256)) -- Read Z2-Linkpointer out of Z2-LINKPOINTER register       
	    --print("Z2LinkPtr: 0x%X" % {linkPointer})
	   		
	   	linkPointer = (linkPointer << 3) & 0xFFFFFFFF -- Bits 31,30 and 29 as most-significant 0 are invalid LinkPointer options
		local bitpos = 28
		local zerofound = 0
		local z2ZoneSelBlockPtr = 0
		while (zerofound == 0) and (bitpos > -1) do
	        if (linkPointer & 0x80000000) == 0 then
		        zerofound = 1
				z2ZoneSelBlockPtr = (0x78200 + ((bitpos + 3)*16 + (i * 1024)))		
			else
				bitpos = bitpos-1
			    linkPointer = linkPointer << 1
			end
		end
		if zerofound == 0 then
			z2ZoneSelBlockPtr = 0x78220 + (i * 1024)
		end
		--print("Z2ZoneSelBlockPtr: 0x%X" % {z2ZoneSelBlockPtr})
		
		--[[
			Perform dummy reads of the Zone Select Block locations */
			0: Zx-EXEONLYRAM
			2: Zx-EXEONLYSECT
			4: Zx-GRABRAM
			6: Zx-GRABSECT
			8: Zx-CSMPSWD0
			A: Zx-CSMPSWD0
			B: Zx-CSMPSWD0
			C: Zx-CSMPSWD0
		--]]
		for j=0,8 do
			readLong(h, z2ZoneSelBlockPtr);
			z2ZoneSelBlockPtr = z2ZoneSelBlockPtr + 2
		end
	end
			
    --Device_Config
	--PARTIDL
	writeLong(h, 0x0005D008, readLong(h, 0x00070200)) -- LOAD PARTIDL Value
	writeLong(h, 0x0005D00A, readLong(h, 0x00070202)) -- LOAD PARTIDH Value

	--DC0 to DC25    
	writeLong(h, 0x0005D010, readLong(h, 0x00070204)) -- LOAD DC0 Value
	writeLong(h, 0x0005D012, readLong(h, 0x00070206)) -- LOAD DC1 Value 
--	    writeLong(h, 0x0005D014, readLong(h, 0x00070208)) -- LOAD DC2 Value 
	writeLong(h, 0x0005D016, readLong(h, 0x0007020A)) -- LOAD DC3 Value 
	writeLong(h, 0x0005D018, readLong(h, 0x0007020C)) -- LOAD DC4 Value
	writeLong(h, 0x0005D01A, readLong(h, 0x0007020E)) -- LOAD DC5 Value
--	    writeLong(h, 0x0005D01C, readLong(h, 0x00070210)) -- LOAD DC6 Value
	writeLong(h, 0x0005D01E, readLong(h, 0x00070212)) -- LOAD DC7 Value
	writeLong(h, 0x0005D020, readLong(h, 0x00070214)) -- LOAD DC8 Value
	writeLong(h, 0x0005D022, readLong(h, 0x00070216)) -- LOAD DC9 Value
	writeLong(h, 0x0005D024, readLong(h, 0x00070218)) -- LOAD DC10 Value
	 writeLong(h, 0x0005D026, readLong(h, 0x0007021A)) -- LOAD DC11 Value
--	    writeLong(h, 0x0005D028, readLong(h, 0x0007021C)) -- LOAD DC12 Value
--	    writeLong(h, 0x0005D02A, readLong(h, 0x0007021E)) -- LOAD DC13 Value
	writeLong(h, 0x0005D02C, readLong(h, 0x00070220)) -- LOAD DC14 Value
	writeLong(h, 0x0005D02E, readLong(h, 0x00070222)) -- LOAD DC15 Value
	writeLong(h, 0x0005D030, readLong(h, 0x00070224)) -- LOAD DC16 Value (Reserved)
	writeLong(h, 0x0005D032, readLong(h, 0x00070226)) -- LOAD DC17 Value
	writeLong(h, 0x0005D034, readLong(h, 0x00070228)) -- LOAD DC18 Value
--	    writeLong(h, 0x0005D036, readLong(h, 0x0007022A)) -- LOAD DC19 Value
	writeLong(h, 0x0005D038, readLong(h, 0x0007022C)) -- LOAD DC20 Value
	writeLong(h, 0x0005D03A, readLong(h, 0x00070230)) -- LOAD DC21 Value
	writeLong(h, 0x0005D03C, readLong(h, 0x00070232)) -- LOAD DC22 Value
	writeLong(h, 0x0005D03E, readLong(h, 0x00070234)) -- LOAD DC23 Value
	writeLong(h, 0x0005D040, readLong(h, 0x00070236)) -- LOAD DC24 Value
	writeLong(h, 0x0005D042, readLong(h, 0x00070238)) -- LOAD DC25 Value  

	-- Load CPUROM_DCx
	writeLong(h, 0x0005D140, readLong(h, 0x00070604)) -- LOAD CPUROM_DC1 Value  
	writeLong(h, 0x0005D142, readLong(h, 0x00070606)) -- LOAD CPUROM_DC2 Value  
	writeLong(h, 0x0005D144, readLong(h, 0x00070608)) -- LOAD CPUROM_DC3 Value  
	writeLong(h, 0x0005D146, readLong(h, 0x0007060A)) -- LOAD CPUROM_DC4 Value  
	   
	-- Load CLAROM_DCx
	writeLong(h, 0x0005D160, readLong(h, 0x00070624)) -- LOAD CLAROM_DC1 Value  
	writeLong(h, 0x0005D162, readLong(h, 0x00070626)) -- LOAD CLAROM_DC2 Value  
	writeLong(h, 0x0005D164, readLong(h, 0x00070628)) -- LOAD CLAROM_DC3 Value  
	writeLong(h, 0x0005D166, readLong(h, 0x0007062A)) -- LOAD CLAROM_DC4 Value  
	   
	-- Load PKGTYPE -  only if KEY is programmed
	if((readLong(h, 0x0007064C) >> 24) == 0x5A) then
	    writeLong(h, 0x0005D12E, readLong(h, 0x0007064C) & 0x00000000F)
    end
	    
	--[[
	    -- C28x_Mode - not sure how to access ST
	    ST1 = ST1 & (~0x0100) -- AMODE = 0
	    ST1 = ST1 | 0x0200 -- OBJMODE = 1
	--]]
	        
    -- OnTargetConnect()			
	writeWord(h, 0x5F412, 0x0003) -- RAM INIT FOR M0/M1 Memory 
	writeWord(h, 0x5F432, 0x00FF) -- RAM INIT FOR LS0..LS7  Memory
	writeWord(h, 0x5F452, 0x000F) -- RAM INIT FOR GS0..GS3 Memory 
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