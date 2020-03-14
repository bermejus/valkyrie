.pragma library

var PIOD_PER = 0x400e1400
var PIOD_OER = 0x400e1410
var PIOD_ODSR = 0x400e1438
var PIOD_OWER = 0x400e14a0
var PIOD_OWDR = 0x400e14a4

function setup(conn)
{
    print("Configuring LED PIO")

    // PIO_D0
    conn.writeu32(PIOD_PER, 0x55)
    conn.writeu32(PIOD_OER, 0x55)
    conn.writeu32(PIOD_OWDR, 0xFFFFFFFF)
    conn.writeu32(PIOD_OWER, 0x55)
}

function red_on(conn)
{
    conn.writeu32(PIOD_ODSR, 0x54)
}

function green_on(conn)
{
    conn.writeu32(PIOD_ODSR, 0x45)
}

function blue_on(conn)
{
    conn.writeu32(PIOD_ODSR, 0x51)
}

function led_on(conn)
{
    conn.writeu32(PIOD_ODSR, 0x15)
}

function on(conn)
{
    conn.writeu32(PIOD_ODSR, 0);
}

function off(conn)
{
    conn.writeu32(PIOD_ODSR, 0x55)
}