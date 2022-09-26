S = {}

local params = {}

function S.setParameter(h, k, v)
   params[k] = v
end

function S.onReset(h)
    if(params['bootmode'] == 'flash') then
         emu.write_memory(h, 0xD00, {0x55aa, 0xB})
    end
end

function S.onDisconnect(h)
	if(params['mode'] == 'nonintrusive') then
		return
	end
    emu.reset(h)   
    emu.runFree(h) 
end

return S