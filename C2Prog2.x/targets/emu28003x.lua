-- emulator script

local S = {}

local dcsm_default_keys_z1 = { -- Z1OTP_CSMPSWD1
  0x4d7fffff, 0x5f7fffff, 0x1dffffff, 0xaf7fffff, 0x1bffffff, 0x17ffffff, 0xbd7fffff, 0x9f7fffff,
  0x2bffffff, 0x27ffffff, 0x7b7fffff, 0xc9ffffff, 0x7d7fffff, 0x6f7fffff, 0x33ffffff
}

local dcsm_default_keys_z2= { -- Z2OTP_CSMPSWD1
  0x1f7fffff, 0xe57fffff, 0x4fffffff, 0xe37fffff, 0x57ffffff, 0x5bffffff, 0xf17fffff, 0x3b7fffff,
  0x8fffffff, 0x6bffffff, 0x377fffff, 0x9bffffff, 0x2f7fffff, 0xcb7fffff, 0x97ffffff,
}

local params = {}

function zx_cr_string(val)
	local cr = ''
	if (val & 0x80000) ~= 0 then
		cr = cr .. 'Allzero, '
	end
	if (val & 0x100000) ~= 0 then
		cr = cr .. 'Allone, '
	end
	if (val & 0x200000) ~= 0 then
		cr = cr .. 'Unsecure, '
	end
	if (val & 0x400000) ~= 0 then
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
    -- read 3 link-pointers (z1 and z2)
	readLong(h, 0x78000)
	readLong(h, 0x78002)
	readLong(h, 0x78004)
	
    readLong(h, 0x78200)
	readLong(h, 0x78202)
	readLong(h, 0x78204)

	-- Read SECDC
	readLong(h, 0x703F0)
	
	-- Z1: JLM_ENABLE, PSWDLOCK
	readLong(h, 0x78006)
    readLong(h, 0x78010)

	-- Z1: CRCLOCK
     readLong(h, 0x78012)
	
	-- Z1: GPREG1-4
    readLong(h, 0x78008)
 	readLong(h, 0x7800A)
 	readLong(h, 0x7800C)
	readLong(h, 0x7800E)
	
	-- Z1: JTAGPSWD
	readLong(h, 0x78014)
	readLong(h, 0x78016)
	
	
	-- Z2: PSWDLOCK
    readLong(h, 0x78210)

	-- Z2: CRCLOCK
    readLong(h, 0x78212)
	
	-- Z2: GPREG1-4
    readLong(h, 0x78208)
 	readLong(h, 0x7820A)
 	readLong(h, 0x7820C)
	readLong(h, 0x7820E)
end

function unblockCsm(h)
  	local cr = readLong(h, 0x5F000+0x18)
	print ("Zone %i status (CR):  0x%X (%s)" % {1, cr, zx_cr_string(cr)})
	cr = readLong(h, 0x5F080+0x18)
	print ("Zone %i status (CR):  0x%X (%s)" % {2, cr, zx_cr_string(cr)})

	activateCsm(h)
	local info = {}
	   	
	-- Z1
    local linkPointer = readLong(h, 0x5F000) -- Read Z1-Linkpointer out of Z1-LINKPOINTER register       
    print("Z1LinkPtr: 0x%X" % {linkPointer})
   		
   	linkPointer = (linkPointer << 18) & 0xFFFFFFFF
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
		z1ZoneSelBlockPtr = 0x78000 + 0x20
	end
	print("Z1ZoneSelBlockPtr: 0x%X" % {z1ZoneSelBlockPtr})	   		
	
	info['zb-1'] = z1ZoneSelBlockPtr
	info['defkey-1'] = dcsm_default_keys_z1[bitpos+2]
	
	-- (GRABSECT1 - EXEONLYRAM1)
	for j=0,8 do
		readLong(h, z1ZoneSelBlockPtr+0x08+j*2);
	end
	-- JTAGPSWD
	readLong(h, z1ZoneSelBlockPtr+0x1C);
	readLong(h, z1ZoneSelBlockPtr+0x1E);
	-- CMPSWD
	for j=0,3 do
		readLong(h, z1ZoneSelBlockPtr+0x00+j*2);
	end
	
	-- Z2
    local linkPointer = readLong(h, 0x5F080) -- Read Z2-Linkpointer out of Z2-LINKPOINTER register       
    print("Z2LinkPtr: 0x%X" % {linkPointer})
   		
   	linkPointer = (linkPointer << 18) & 0xFFFFFFFF
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
		z2ZoneSelBlockPtr = 0x78200 + 0x20
	end
	print("Z2ZoneSelBlockPtr: 0x%X" % {z2ZoneSelBlockPtr})
		   		
	info['zb-2'] = z2ZoneSelBlockPtr
	info['defkey-2'] = dcsm_default_keys_z2[bitpos+2]
	
	-- (GRABSECT1 - EXEONLYRAM1)
	for j=0,8 do
		readLong(h, z2ZoneSelBlockPtr+0x08+j*2);
	end
	-- CMPSWD
	for j=0,3 do
		readLong(h, z2ZoneSelBlockPtr+0x00+j*2);
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
        pwdlock = readLong(h, 0x5F082)
		lnkbase = 0x5F080
        keybase = 0x5F090
    end

	print ("Zone %i PSWDLOCK:  0x%X" % {zone, pwdlock})

    if((pwdlock & 0xF0) == 0xF0) then
        local pwds = readLongBlock(h, zinfo['zb-%i' % zone]+0x0, 4)
        info['keys'] = pwds
         -- unlock
        writeLongBlock(h, keybase, pwds)
    else
        -- attempt to unlock with defaults
        local pwds = {0xFFFFFFFF, zinfo['defkey-%i' % {zone}], 0xFFFFFFFF, 0xFFFFFFFF}
        writeLongBlock(h, keybase, pwds)
        if((readLong(h, lnkbase+0x18) & 0x200000) == 0) then
            if keys ~= nil then
                -- try custom keys supplied
                writeLongBlock(h, keybase, keys)
                if((readLong(h, lnkbase+0x18) & 0x190000) == 0) then
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
	    
	    local cr = readLong(h, 0x5F000+0x18)
		print ("Zone %i status (CR):  0x%X (%s)" % {1, cr, zx_cr_string(cr)})
		cr = readLong(h, 0x5F080+0x18)
		print ("Zone %i status (CR):  0x%X (%s)" % {2, cr, zx_cr_string(cr)})
	    
	    print('Zone base 1 0x%X and default key 0x%X' % {zinfo['zb-1'], zinfo['defkey-1']})
	    print('Zone base 2 0x%X and default key 0x%X' % {zinfo['zb-2'], zinfo['defkey-2']})
		    
   		writeWord(h, 0x0005F400 + 0x12, 0x0003) -- RAM INIT FOR M0/M1 Memory
		writeWord(h, 0x0005F400 + 0x32, 0x00FF) -- RAM INIT FOR LS1..LS7  Memory
		writeWord(h, 0x0005F400 + 0x52, 0x000F) -- RAM INIT FOR GS0..GS3 Memory  
								
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