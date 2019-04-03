#include <ros/ros.h>

#include <ros_ws281x/SetGamma.h>
#include <ros_ws281x/SetLeds.h>

#include <ws2811.h>

#include <ros/console.h>

#include <unordered_map>

#include <signal.h>

std::unordered_map<std::string, uint64_t> ws2811Types =
        {
                {"SK6812_STRIP_RGBW", SK6812_STRIP_RGBW},
                {"SK6812_STRIP_RBGW", SK6812_STRIP_RBGW},
                {"SK6812_STRIP_GRBW", SK6812_STRIP_GRBW},
                {"SK6812_STRIP_GBRW", SK6812_STRIP_GBRW},
                {"SK6812_STRIP_BRGW", SK6812_STRIP_BRGW},
                {"SK6812_STRIP_BGRW", SK6812_STRIP_BGRW},
                {"WS2811_STRIP_RGB", WS2811_STRIP_RGB},
                {"WS2811_STRIP_RBG", WS2811_STRIP_RBG},
                {"WS2811_STRIP_GRB", WS2811_STRIP_GRB},
                {"WS2811_STRIP_GBR", WS2811_STRIP_GBR},
                {"WS2811_STRIP_BRG", WS2811_STRIP_BRG},
                {"WS2811_STRIP_BGR", WS2811_STRIP_BGR},
                {"WS2812_STRIP", WS2812_STRIP},
                {"SK6812_STRIP", SK6812_STRIP},
                {"SK6812W_STRIP", SK6812W_STRIP}
        };


ws2811_t ledString;
bool didInitialize = false;

bool setGamma(ros_ws281x::SetGamma::Request& req, ros_ws281x::SetGamma::Response& resp)
{
    for(int i = 0; i < 255; ++i)
    {
        ledString.channel[0].gamma[i] = req.gamma[i];
    }
    resp.success = 1;
    return true;
}

bool setLeds(ros_ws281x::SetLeds::Request& req, ros_ws281x::SetLeds::Response& resp)
{
    size_t maxLed = std::min(req.colors.size(), (size_t)ledString.channel[0].count);
    for(size_t i = 0; i < maxLed; ++i)
    {
        auto color = uint32_t(
                0x00010000 * int(req.colors[i].r) +  // Red channel mask
                0x00000100 * int(req.colors[i].g) +  // Green channel mask
                0x00000001 * int(req.colors[i].b) +  // Blue channel mask
                0x01000000 * int(req.colors[i].a));  // Use alpha for white
        ledString.channel[0].leds[i] = color;
    }
    ws2811_return_t ret;
    if ((ret = ws2811_render(&ledString)) != WS2811_SUCCESS)
    {
        resp.message = ws2811_get_return_t_str(ret);
        ROS_ERROR_THROTTLE(1, "[ros_ws281x] Could not set LED colors: %s", resp.message.c_str());
        resp.success = 0;
    }
    else
    {
        resp.success = 1;
        resp.message = "";
    }
    return true;
}

void cleanup(int signal)
{
    (void) signal;
    if (didInitialize)
    {
        for(int i = 0; i < ledString.channel[0].count; ++i)
        {
            ledString.channel[0].leds[i] = 0;
            ws2811_render(&ledString);
        }
        ws2811_fini(&ledString);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_ws281x");
    ros::NodeHandle nh, nh_priv("~");

    int paramFreq;
    int paramPin;
    int paramDma;
    uint64_t paramStripType;
    int paramLedCount;
    bool paramInvert;
    int paramBrightness;

    nh_priv.param("target_frequency", paramFreq, WS2811_TARGET_FREQ);
    nh_priv.param("gpio_pin", paramPin, 21);
    nh_priv.param("dma", paramDma, 10);

    std::string stripTypeStr;
    nh_priv.param("strip_type", stripTypeStr, std::string("WS2811_STRIP_GBR"));
    nh_priv.param("led_count", paramLedCount, 30);
    nh_priv.param("invert", paramInvert, false);
    nh_priv.param("brightness", paramBrightness, 255);

    auto stripTypeIt = ws2811Types.find(stripTypeStr);
    if (stripTypeIt != ws2811Types.end())
    {
        paramStripType = stripTypeIt->second;
    }
    else
    {
        ROS_WARN("[ros_ws281x] Unknown strip type: %s", stripTypeStr.c_str());
        paramStripType = WS2811_STRIP_GBR;
    }

    if (paramFreq < 0)
    {
        ROS_WARN("[ros_ws281x] Target_frequency out of range, resetting to default");
        ledString.freq = (uint32_t)WS2811_TARGET_FREQ;
    }
    else
    {
        ledString.freq = (uint32_t)paramFreq;
    }

    ledString.dmanum = paramDma;
    ledString.channel[0].gpionum = paramPin;
    ledString.channel[0].count = paramLedCount;
    ledString.channel[0].invert = paramInvert ? (1) : (0);
    ledString.channel[0].brightness = (uint8_t)paramBrightness;
    ledString.channel[0].strip_type = (int)paramStripType;

    // Disable second channel for now
    ledString.channel[1].gpionum = 0;
    ledString.channel[1].count = 0;
    ledString.channel[1].invert = 0;
    ledString.channel[1].brightness = 0;

    ws2811_return_t ret;
    if ((ret = ws2811_init(&ledString)) != WS2811_SUCCESS)
    {
        ROS_FATAL("[ros_ws281x] native library init failed: %s", ws2811_get_return_t_str(ret));
        exit(1);
    }
    didInitialize = true;
    signal(SIGINT, cleanup);

    auto srvGamma = nh.advertiseService("set_gamma", setGamma);
    auto srvLeds = nh.advertiseService("set_leds", setLeds);

    ros::spin();

    return 0;
}
