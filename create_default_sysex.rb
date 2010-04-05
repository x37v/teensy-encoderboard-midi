NUM_BOARDS = 2
SYSEX_FILE = "default.syx"
#dev, buzzr, 0
SYSEX_HEADER = [125, 98, 117, 122, 122, 114, 0]
SYSEX_BEGIN = 0xF0
SYSEX_END = 0xF7

SET_ENCODER_DATA = 5
SET_BUTTON_DATA = 6
SET_ADC_DATA = 7

ENC_ABSOLUTE = 0x1
ENC_DETENT_ONLY = 0x2
ENC_BUTTON_MUL = 0x10

File.open(SYSEX_FILE, "w"){ |f|
  #encoders
  (NUM_BOARDS * 8).times do |i|
    f.print SYSEX_BEGIN.chr
    SYSEX_HEADER.each do |h|
      f.print h.chr
    end
    f.print SET_ENCODER_DATA.chr
    #index
    f.print i.chr
    #flag
    flags = ENC_DETENT_ONLY | ENC_BUTTON_MUL
    f.print flags.chr
    #chan
    f.print 0.chr
    #cc num
    f.print i.chr
    #btn chan
    f.print 0.chr
    #bun num/mul
    if flags & ENC_BUTTON_MUL != 0
      f.print 4.chr
    else
      f.print((i + (NUM_BOARDS * 12)).chr)
    end
    f.print SYSEX_END.chr
  end
  #buttons
  (NUM_BOARDS * 4).times do |i|
    f.print SYSEX_BEGIN.chr
    SYSEX_HEADER.each do |h|
      f.print h.chr
    end
    f.print SET_BUTTON_DATA.chr
    f.print i.chr
    f.print 0.chr
    f.print((i + NUM_BOARDS * 8).chr)
    f.print SYSEX_END.chr
  end
}
