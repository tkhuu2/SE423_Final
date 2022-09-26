S = {}

local params = {}

function writeLong(h, address, long)
	emu.write_memory(h, address, {long & 0xFF, (long >> 8) & 0xFF, (long >> 16) & 0xFF, (long >> 24) & 0xFF})
end

function readLong(h, address)
	local mem = emu.read_memory(h, address, 4)
	return mem[1] + (mem[2] << 8) + (mem[3] << 16) + (mem[4] << 24)
end

function S.setParameter(h, k, v)
   params[k] = v
end

function S.onReset(h)
--	print("Emulator reset.")
	
	if((readLong(h, 0x400FE05C) & 0x01) == 1) then
		writeLong(h, 0x400FA048, 0x1)
		writeLong(h, 0x400FA040, 0x3)
		-- should wait here...
	end
	
	writeLong(h, 0x400FB980, 0xA5A5A5A5) -- disable protection MWRALLOW
	
	writeLong(h, 0x400FA600, 0x0) -- disable flash ECC
		
	local status = readLong(h, 0x400FB480)
	writeLong(h, 0x400FB480, status | 0x8000) -- FOCESEC bit in Z1_CSMCR
							
	readLong(h, 0x681000) -- dummy read to OTPSEC	
		
	-- unlock CSMZ1
	readLong(h, 0x200000) -- dummy reads
	readLong(h, 0x200004)
	readLong(h, 0x200008)
	readLong(h, 0x20000C)

	writeLong(h, 0x400FB400, 0xFFFFFFFF) -- write passwords
	writeLong(h, 0x400FB404, 0xFFFFFFFF)
	writeLong(h, 0x400FB408, 0xFFFFFFFF)
	writeLong(h, 0x400FB40C, 0xFFFFFFFF)
				
	-- unlock ECSKZ1
	readLong(h, 0x200010) -- dummy reads
	readLong(h, 0x200014) -- dummy reads
	writeLong(h, 0x400FB410, 0xFFFFFFFF) -- write passwords
	writeLong(h, 0x400FB414, 0xFFFFFFFF)
				
	readLong(h, 0x200018) -- Z1 GRABSECT
	readLong(h, 0x20001C) -- Z1 GRABRAM
	readLong(h, 0x200020) -- read flash EXEZ1	
		
	-- unlock CSMZ2
	readLong(h, 0x2FFFF0) -- dummy reads
	readLong(h, 0x2FFFF4)
	readLong(h, 0x2FFFF8)
	readLong(h, 0x2FFFFC)
	writeLong(h, 0x400FB418, 0xFFFFFFFF) -- write passwords
	writeLong(h, 0x400FB41C, 0xFFFFFFFF)
	writeLong(h, 0x400FB420, 0xFFFFFFFF)
	writeLong(h, 0x400FB424, 0xFFFFFFFF)
		
	-- unlock ECSLZ2
	readLong(h, 0x2FFFE8) -- dummy reads
	readLong(h, 0x2FFFEC) -- dummy reads
	writeLong(h, 0x400FB428, 0xFFFFFFFF) -- write passwords
	writeLong(h, 0x400FB42C, 0xFFFFFFFF)
		
	-- grab sectors	
	readLong(h, 0x2FFFDC) -- read flash EXEZ2
	readLong(h, 0x2FFFE4) -- Z2 GRABSECT
	readLong(h, 0x2FFFE0) -- Z2 GRABRAM	
	
	-- device configuration
	writeLong(h, 0x400FE004, readLong(h, 0x00680430))
	writeLong(h, 0x400FB900, readLong(h, 0x00680410))
	writeLong(h, 0x400FB904, readLong(h, 0x00680414))
	writeLong(h, 0x400FB930, readLong(h, 0x0068042C))
                                     
	writeLong(h, 0x400FE014, readLong(h, 0x00680400)) 
	writeLong(h, 0x400FE01C, readLong(h, 0x00680404))
	writeLong(h, 0x400FE024, readLong(h, 0x00680408)) 
	writeLong(h, 0x400FE194, readLong(h, 0x0068040C)) 
	writeLong(h, 0x400FB910, readLong(h, 0x00680418)) 
	writeLong(h, 0x400FB914, readLong(h, 0x0068041C)) 
	writeLong(h, 0x400FB918, readLong(h, 0x00680420)) 
	writeLong(h, 0x400FB91C, readLong(h, 0x00680424)) 
	writeLong(h, 0x400FB920, readLong(h, 0x00680428)) 
	writeLong(h, 0x400FE308, readLong(h, 0x00680434)) 
    
	writeLong(h, 0x400FB8C0,  readLong(h, 0x400FB8C0) | 0x00030001) -- Release C28 from Reset   
		
	-- initialize RAM
	writeLong(h, 0x400FB240, 0x55) -- Initialize C0, C1, C2, C3 RAMs	
	writeLong(h, 0x400FB210, 0x00) -- assign Sx RAMs to M3		
	writeLong(h, 0x400FB250, 0x5555) -- initialize Sx RAMs		
end

return S