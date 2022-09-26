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

function S.setParameter(h, k, v)
   params[k] = v
end

function S.onReset(h)
    --5aFF or 5a0b
    if(params['bootmode'] == 'flash') then
        emu.write_memory(h, 0xD00, {0xFF5a, 0x0000})
        return
    end
	
	writeWord(0x5D122, 0xA5A50000) -- release CPU2
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
	  
	readLong(h, 0x703F0) -- Read SECDC   
	   
	readLong(h, 0x78010) -- Read Z1 PSWDLOCK
	readLong(h, 0x78014) -- Read Z1 CRCLOCK
	readLong(h, 0x78018) -- Read Z1 JTAGLOCK
	readLong(h, 0x7801E) -- Read Z1 BOOTMODE

	--readLong(h, 0x7800C) -- Read Z1 GPREG1
	--readLong(h, 0x7800E) -- Read Z1 GPREG2
	--readLong(h, 0x7801C) -- Read Z1 GPREG3

	readLong(h, 0x78210) -- Read Z2 PSWDLOCK
	readLong(h, 0x78214) -- Read Z2 CRCLOCK
	readLong(h, 0x78218) -- Read Z2 JTAGLOCK
	readLong(h, 0x7821E) -- Read Z2 BOOTMODE

	--readLong(h, 0x7820C) -- Read Z2 GPREG1
	--readLong(h, 0x7820E) -- Read Z2 GPREG2
	--readLong(h, 0x7821C) -- Read Z2 GPREG3
	   	
	for i=0,0 do
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
		--print("Z2ZoneSelBlockPtr: 0x%X" %  {z2ZoneSelBlockPtr})	   		
		
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
				        
    -- OnTargetConnect()			
	writeWord(h, 0x5F412, 0x000F) -- RAM INIT FOR M0/M1/D0/D1 Memory
	writeWord(h, 0x5F432, 0x003F) -- RAM INIT FOR LS0..LS5  Memory
	writeWord(h, 0x5F452, 0xFFFF) -- RAM INIT FOR GS0..GS15 Memory 
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