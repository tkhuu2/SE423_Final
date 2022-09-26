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
	--print("Emulator reset.")
	
	writeWord(h, 0x7029, readWord(h, 0x7029) | 0x68) --  Disable WD
	writeWord(h, 0x7025, 0x0055)
	writeWord(h, 0x7025, 0x00AA)
	
	-- sequence is important!
	readLong(h, 0x3D7A00) -- Read Z1 Linkpointer 
	readLong(h, 0x3D7800) -- Read Z2 Linkpointer 
	readLong(h, 0x3D7A02) -- Read Z1 OTPSECLOCK
	readLong(h, 0x3D7802) -- Read Z2 OTPSECLOCK
	readLong(h, 0x3D7A04) -- Read Z1 BOOTMODE
	readLong(h, 0x3D7804) -- Read Z2 BOOTMODE	
	readLong(h, 0x3D7FFE) -- Read SECDC
	   	
    local linkPointer = readLong(h, 0xB80) -- Read Z1-Linkpointer out of Z1-LINKPOINTER register       
    --print("Z1LinkPtr: 0x%X" % {linkPointer})
   		
   	linkPointer = (linkPointer << 2) & 0xFFFFFFFF -- Bits 31,30 and 29 as most-significant 0 are invalid LinkPointer options
	local bitpos = 29
	local zerofound = 0
	local z1ZoneSelBlockPtr = 0
	while (zerofound == 0) and (bitpos > -1) do
        if (linkPointer & 0x80000000) == 0 then
	        zerofound = 1
			z1ZoneSelBlockPtr = (0x3D7A00 + ((bitpos + 2)*16))		
		else
			bitpos = bitpos-1
		    linkPointer = linkPointer << 1
		end
	end
	if zerofound == 0 then
		z1ZoneSelBlockPtr = 0x3D7A10
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
	
    local linkPointer = readLong(h, 0xBC0) -- Read Z2-Linkpointer out of Z2-LINKPOINTER register       
    --print("Z2LinkPtr: 0x%X" % {linkPointer})
   		
   	linkPointer = (linkPointer << 2) & 0xFFFFFFFF -- Bits 31,30 and 29 as most-significant 0 are invalid LinkPointer options
	local bitpos = 29
	local zerofound = 0
	local z2ZoneSelBlockPtr = 0
	while (zerofound == 0) and (bitpos > -1) do
        if (linkPointer & 0x80000000) == 0 then
	        zerofound = 1
			z2ZoneSelBlockPtr = (0x3D7800 + ((bitpos + 2)*16))		
		else
			bitpos = bitpos-1
		    linkPointer = linkPointer << 1
		end
	end
	if zerofound == 0 then
		z2ZoneSelBlockPtr = 0x3D7810
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
	
	-- this is how the Z1 password would be written
	--[[
	writeLong(h, 0xB90, 0xFFFFFFFF)
	writeLong(h, 0xB92, 0xFFFFFFFF)
	writeLong(h, 0xB94, 0xFFFFFFFF)
	writeLong(h, 0xB96, 0xFFFFFFFF)		
	if((readWord(h, 0xB99) & 0x0020) ~= 0x0020) then
		print("Zone 1 is secure!")
	else
		print("Zone 1 is unsecure.")
    end
    --]]
	
	-- System.out.println("Z1 Forcesec="+Integer.toBinaryString(readWord(0xB99)));

	-- this is how the Z2 password would be written
	--[[
	writeLong(h, 0xBD0, 0xFFFFFFFF)
	writeLong(h, 0xBD2, 0xFFFFFFFF)
	writeLong(h, 0xBD4, 0xFFFFFFFF)
	writeLong(h, 0xBD6, 0xFFFFFFFF)		
	if((readWord(h, 0xBD9) & 0x0020) ~= 0x0020) then
		print("Zone 2 is secure!")
	else
		print("Zone 2 is unsecure.")
    end
    --]]

	-- System.out.println("Z2 Forcesec="+Integer.toBinaryString(readWord(0xBD9)));	
	
	-- configure device
	writeWord(h, 0x0986, (readWord(h, 0x0986)  & ~(0x1FF0)) | (readWord(h, 0x003D7FDF) &  0x1FF0));
	writeLong(h, 0x886, readLong(h, 0x003D7FD0));
	writeLong(h, 0x888, readLong(h, 0x003D7FD2));
	writeLong(h, 0x88A, readLong(h, 0x003D7FD4));		
end

return S