-- emulator script

local S = {}

local dcsm_default_keys_z1 = { -- Z1OTP_CSMPSWD1
  0x47ffffff, 0xdb7fffff, 0x4bffffff, 0x3f7fffff, 0xcfbfffff, 0x8bffffff, 0x53ffffff, 0xcf7fffff, 
  0xe77fffff, 0x93ffffff, 0xeb7fffff, 0x69ffffff, 0xa9ffffff, 0xdd7fffff, 0x8bffffff, 0xcfbfffff, 
  0x3f7fffff, 0x4bffffff, 0xdb7fffff, 0x47ffffff, 0x87ffffff, 0xf37fffff, 0xdd7fffff, 0xa9ffffff, 
  0x69ffffff, 0xeb7fffff, 0x93ffffff, 0xe77fffff, 0xcf7fffff
}

local dcsm_default_keys_z2= { -- Z2OTP_CSMPSWD1
  0xe3ffffff, 0x977fffff, 0xf1ffffff, 0x9b7fffff, 0x5b7fffff, 0x2fffffff, 0x1fffffff, 0x6b7fffff, 
  0xab7fffff, 0x37ffffff, 0x4f7fffff, 0x3bffffff, 0xe5ffffff, 0x8f7fffff, 0x2fffffff, 0x5b7fffff,
  0x9b7fffff, 0xf1ffffff, 0x977fffff, 0xe3ffffff, 0xcbffffff, 0x577fffff, 0x8f7fffff, 0xe5ffffff, 
  0x3bffffff, 0x4f7fffff, 0x37ffffff, 0xab7fffff, 0x6b7fffff, 0x1fffffff
}

local params = {}

function zx_cr_string(val)
	local cr = ''
	if (val & 0x8) ~= 0 then
		cr = cr .. 'Allzero, '
	end
	if (val & 0x10) ~= 0 then
		cr = cr .. 'Allone, '
	end
	if (val & 0x20) ~= 0 then
		cr = cr .. 'Unsecure, '
	end
	if (val & 0x40) ~= 0 then
		cr = cr .. 'Armed, '
	end
	if cr ~= '' then
		cr = cr:sub(1, -3)
	end
	return cr
end

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

function writeLongBlock(h, address, mem)
	for i=1,#mem do
		writeLong(h, address+2*(i-1), mem[i])
	end
end

function readLongBlock(h, address, len)
	mem = {}
	for i=1,len do
		mem[i] = readLong(h, address+2*(i-1))
	end
	return mem
end

function activateCsm(h)
	readLong(h, 0x78000)
	readLong(h, 0x78004)
	readLong(h, 0x78008)
	
	readLong(h, 0x78200)
	readLong(h, 0x78204)
	readLong(h, 0x78208)
	
	-- Read SECDC
	readLong(h, 0x703F0)
		
	-- Z1: PSWDLOCK, CRCLOCK
	readLong(h, 0x78010)
	readLong(h, 0x78014)
	
	-- Z1: reserved
	readLong(h, 0x78018)
	
	-- Z1: GPREG1-3
	readLong(h, 0x7800C)
	readLong(h, 0x7800E)
	readLong(h, 0x7801C)
		
	-- Z1: Z1OTP_BOOTCTRL
	readLong(h, 0x7801E)
	
	-- Z2: PSWDLOCK, CRCLOCK
	readLong(h, 0x78210)
	readLong(h, 0x78214)
	
	-- Z2: reserved
	readLong(h, 0x78218)
	
	-- Z2: Z2OTP_BOOTCTRL
	readLong(h, 0x7821E)
	
	-- Z2: GPREG1-3
	readLong(h, 0x7820C)
	readLong(h, 0x7820E)
	readLong(h, 0x7821C)
end

function unblockCsm(h)
  	local cr = readLong(h, 0x5F000+0x19)
	print ("Zone %i status (CR):  0x%X (%s)" % {1, cr, zx_cr_string(cr)})
	cr = readLong(h, 0x5F040+0x19)
	print ("Zone %i status (CR):  0x%X (%s)" % {2, cr, zx_cr_string(cr)})

	activateCsm(h)
	local info = {}
	   	
	for i=0,0 do
	    local linkPointer = readLong(h, 0x5F000 + (i * 256)) -- Read Z1-Linkpointer out of Z1-LINKPOINTER register       
	    print("Z1LinkPtr: 0x%X" % {linkPointer})
	   		
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
			z1ZoneSelBlockPtr = 0x78000 + 0x20 + (i * 1024)
		end
		print("Z1ZoneSelBlockPtr: 0x%X" % {z1ZoneSelBlockPtr})	   		
		
		info['zb-1'] = z1ZoneSelBlockPtr
		info['defkey-1'] = dcsm_default_keys_z1[bitpos+2]
		
		for j=0,8 do
			readLong(h, z1ZoneSelBlockPtr);
			z1ZoneSelBlockPtr = z1ZoneSelBlockPtr + 2
		end
	end
	
	for i=0,0 do
	    local linkPointer = readLong(h, 0x5F040 + (i * 256)) -- Read Z2-Linkpointer out of Z2-LINKPOINTER register       
	    print("Z2LinkPtr: 0x%X" % {linkPointer})
	   		
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
			z2ZoneSelBlockPtr = 0x78200 + 0x20 + (i * 1024)
		end
		print("Z2ZoneSelBlockPtr: 0x%X" % {z2ZoneSelBlockPtr})
			   		
		info['zb-2'] = z2ZoneSelBlockPtr
		info['defkey-2'] = dcsm_default_keys_z2[bitpos+2]
		
		for j=0,8 do
			readLong(h, z2ZoneSelBlockPtr);
			z2ZoneSelBlockPtr = z2ZoneSelBlockPtr + 2
		end
	end
	return info
end

function unlock_zone(h, zone, zinfo, keys)
    local info = {}

    local pwdlock, keybase
    if zone == 1 then
        pwdlock = readLong(h, 0x5F002)
	    lnkbase = 0x5F000
        keybase = 0x5F010
    else
        pwdlock = readLong(h, 0x5F042)
		lnkbase = 0x5F040
        keybase = 0x5F050
    end

	print ("Zone %i PSWDLOCK:  0x%X" % {zone, pwdlock})

    if((pwdlock & 0xF0) == 0xF0) then
        local pwds = readLongBlock(h, zinfo['zb-%i' % zone]+0x8, 4)
        info['keys'] = pwds
         -- unlock
        writeLongBlock(h, keybase, pwds)
    else
        -- attempt to unlock with defaults
        local pwds = {0xFFFFFFFF, zinfo['defkey-%i' % {zone}], 0xFFFFFFFF, 0xFFFFFFFF}
        writeLongBlock(h, keybase, pwds)
        if((readWord(h, keybase+9) & 0x20) == 0) then
            if keys ~= nil then
                -- try custom keys supplied
                writeLongBlock(h, keybase, keys)
                if((readWord(h, keybase+9) & 0x19) == 0) then
                    print('Key match.')
                    info['keys'] = keys
                end
            end
        else
            info['keys'] = pwds
        end
    end
    return info
end

function S.setParameter(h, k, v)
   params[k] = v
end

function S.onReset(h)
    if(params['bootmode'] == 'flash') then
    	print('Configure boot from flash...')
		writeLong(h, 0xD00, 0x5AFFFFFF) 
    	writeLong(h, 0xD04, 0x0003)
		return

    elseif (params['bootmode'] == 'ram') then
		print('Boot from RAM...')
    	
    	writeWord(0x5FB00, 0) -- disable Flash ECC
		writeWord(0x7029, 0x68) -- disable WD	
		
	    zinfo = unblockCsm(h)
	    
	    local cr = readLong(h, 0x5F000+0x19)
		print ("Zone %i status (CR):  0x%X (%s)" % {1, cr, zx_cr_string(cr)})
		cr = readLong(h, 0x5F040+0x19)
		print ("Zone %i status (CR):  0x%X (%s)" % {2, cr, zx_cr_string(cr)})
	    
	    print('Zone base 1 0x%X and default key 0x%X' % {zinfo['zb-1'], zinfo['defkey-1']})
	    print('Zone base 2 0x%X and default key 0x%X' % {zinfo['zb-2'], zinfo['defkey-2']})

		writeLong(h, 0x5F082, 0x5A5A0001)
		stat = readLong(h, 0x5F080)
		while stat ~= 1 do
			stat = readLong(h, 0x5F080)
		end 	    

		writeWord(h, 0xA000, 0xAA55)
		if readWord(h, 0xA000) ~= 0xAA55 then
			print('Chip requires early unlocking.')
			 
			unlock_zone(h, 1, zinfo)
			unlock_zone(h, 2, zinfo)
			 
			local cr = readLong(h, 0x5F000+0x19)
			print ("Zone %i status (CR):  0x%X (%s)" % {1, cr, zx_cr_string(cr)})
			cr = readLong(h, 0x5F040+0x19)
			print ("Zone %i status (CR):  0x%X (%s)" % {2, cr, zx_cr_string(cr)})
		end
		    
   		writeWord(h, 0x0005F400 + 0x12, 0x0003) -- RAM INIT FOR M0/M1 Memory
		writeWord(h, 0x0005F400 + 0x32, 0x00F0) -- RAM INIT FOR LS4..LS7  Memory
		writeWord(h, 0x0005F400 + 0x52, 0x0010) -- RAM INIT FOR GS0. Memory  
								
		writeWord(h, 0xA000, 0xAA55)
		if readWord(h, 0xA000) ~= 0xAA55 then
			error('Unable to open LS RAM.')
		end
    	
    	-- configure boot from SRAM
   		writeLong(h, 0xD00, 0x5AFFFFFF) 
    	writeLong(h, 0xD04, 0x0005)
    else
    	 -- "custom" bootmode
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