S = {}

local params = {}

function S.setParameter(h, k, v)
   params[k] = v
end

function S.onReset(h)
    if(params['bootmode'] == 'flash') then
        --print("Emulator reset. Configure boot from flash.")
        emu.write_memory(h, 0xD00, {0x55aa, 0xB})
        --[[
        print('Unlocking CSM')
        local mem = emu.read_memory(h, 0x3F7FF8, 8)
       
        emu.write_memory(h, 0x8c00, {10})
        local mem = emu.read_memory(h, 0x8c00, 1)
        print (mem[1])
        --]]
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