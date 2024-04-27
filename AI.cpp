#include <vector>
#include <thread>
#include <array>
#include <map>
#include "AI.h"
#include "constants.h"
// 注意不要使用conio.h，Windows.h等非标准库
// 为假则play()期间确保游戏状态不更新，为真则只保证游戏状态在调用相关方法时不更新，大致一帧更新一次
extern const bool asynchronous = false;

// 选手需要依次将player1到player4的船类型在这里定义
extern const std::array<THUAI7::ShipType, 4> ShipTypeDict = {
    THUAI7::ShipType::CivilianShip,
    THUAI7::ShipType::MilitaryShip,
    THUAI7::ShipType::MilitaryShip,
    THUAI7::ShipType::FlagShip,
};

// 可以在AI.cpp内部声明变量与函数
bool attack = false;
bool fuck = false;
bool constructedFactory = false;

void AI::play(IShipAPI& api)
{
    if (this->playerID == 1)
    {
        auto info = api.GetSelfInfo();
        int x = info->x, y = info->y;
        if (!attack)
        {
            auto res = api.MoveLeft((y - 41.5*1000)/3.0);
            res.wait();
            //auto info = api.GetSelfInfo();
            //std::cout << info->x << ' ' << info->y << std::endl; 
            //std::cout << (13 * 1000 - x) / 3.0 << std::endl;
            //res = api.MoveDown((13*1000-x)/3.0);
            //res.wait();
            attack = true;
        }
        if (attack)
        {
            auto info = api.GetSelfInfo();
            x = info->x;
            std::cout << (14 * 1000 - x) / 3.0 << std::endl;
            auto res = api.MoveDown((14*1000-x)/3.0);
            res.wait();
            info = api.GetSelfInfo();
            std::cout << info->x << ' ' << info->y << std::endl; 
            fuck = true; 
        }
        if (fuck)
        {
            auto info = api.GetSelfInfo();
            std::cout << info->x << ' ' << info->y << std::endl; 
            x = info->x;
            if (x >= 13 * 1000)
            {
            auto res = api.MoveRight(7 * 1000/ 3.0);
            res.wait();
            fuck = false; 
            
            }
        }
        if (!constructedFactory)
        {
        //auto res = api.Construct(THUAI7::ConstructionType::Factory);
            auto res = api.Produce();
        bool result = res.get();
        if(result) constructedFactory = true;
        }
    }
    else if (this->playerID == 2)
    {
    }

    else if (this->playerID == 3)
    {
    }
    else if (this->playerID == 4)
    {
        api.MoveDown(100);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        api.MoveLeft(100);
    }
}

void AI::play(ITeamAPI& api)  // 默认team playerID 为0
{
//    api.PrintSelfInfo();
//    std::this_thread::sleep_for(std::chrono::seconds(1));
//    api.InstallModule(2, THUAI7::ModuleType::ModuleLaserGun);
    auto map = api.GetFullMap();
    for (int i = 0; i < map.size(); i++)
    {
        for (int j = 0; j < map.size(); j++)
        {
            if (map[i][j] == THUAI7::PlaceType::Resource)
            {
                std::cout << i << ' '<< j << std::endl;
            }
        }
    }
}
