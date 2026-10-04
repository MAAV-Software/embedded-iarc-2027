local FLIGHT_MODE = 0           -- flight mode to write to (FM0)
local MAX_GV_INDEX = 8          -- EdgeTX has GV1..GV9 -> indexes 0..8

local pending = {}              -- bytes waiting to form a 2-byte message
local count = 0                 -- total bytes received
local badCount = 0              -- bytes discarded while resyncing
local lastIdx, lastVal = nil, nil

-- word = hi*256 + lo
--   bits 0..11  : value + 1024 (0..2048)
--   bits 12..15 : variable index (0..9)
local function decode(lo, hi)
    local word  = hi * 256 + lo
    local raw   = word % 4096
    local index = math.floor(word / 4096)
    return index, raw - 1024, raw
end

local function isValid(lo, hi)
    local index, _, raw = decode(lo, hi)
    return raw <= 2048 and index <= 9
end

local function handleMessage(lo, hi)
    local index, value = decode(lo, hi)
    lastIdx, lastVal = index, value

    if index <= MAX_GV_INDEX then
        model.setGlobalVariable(index, FLIGHT_MODE, value)
    end
end

local function run(event)
    local data = serialRead()

    if data ~= nil and #data > 0 then
        count = count + #data

        for i = 1, #data do
            pending[#pending + 1] = string.byte(data, i)

            if #pending == 2 then
                local lo, hi = pending[1], pending[2]
                if isValid(lo, hi) then
                    handleMessage(lo, hi)
                    pending = {}
                else
                    -- misaligned: drop oldest byte, keep the newer one
                    badCount = badCount + 1
                    pending = { hi }
                end
            end
        end
    end

    lcd.clear()
    lcd.drawText(5, 5, "USB-VCP LUA test")
    lcd.drawText(5, 20, "Bytes: " .. count .. " Bad: " .. badCount)
    if lastIdx ~= nil then
        lcd.drawText(5, 35, "GV" .. (lastIdx + 1) .. " FM" .. FLIGHT_MODE)
        lcd.drawText(5, 50, "Val: " .. lastVal)
    end

    if event == EVT_VIRTUAL_EXIT then
        return 1
    end

    return 0
end

return { run = run }