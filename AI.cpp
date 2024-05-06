#include <vector>
#include <thread>
#include <array>
#include <queue>
#include <list>
#include <unordered_set>
#include <cstring>
#include "AI.h"
#include "constants.h"
#define maxLen 55
// 注意不要使用conio.h，Windows.h等非标准库
// 为假则play()期间确保游戏状态不更新，为真则只保证游戏状态在调用相关方法时不更新，大致一帧更新一次
extern const bool asynchronous = false;
#ifndef PI
#define PI 3.14159265358979323846
#endif  // !PI


// 选手需要依次将player1到player4的船类型在这里定义
extern const std::array<THUAI7::ShipType, 4> ShipTypeDict = {
    THUAI7::ShipType::CivilianShip,
    THUAI7::ShipType::CivilianShip,
    THUAI7::ShipType::MilitaryShip,
    THUAI7::ShipType::FlagShip,
};
 
// 可以在AI.cpp内部声明变量与函数


struct coordinate
{
    int x, y;
    bool operator==(const coordinate& c) const
    {
        return x == c.x and y == c.y;
    }
    bool operator<(const coordinate& c) const
    {
        if (x < c.x)
            return true;
        if (x > c.x)
            return false;
        return y < c.y;
    }
};
struct PointHash
{
    std::size_t operator()(const coordinate& c) const
    {
        return std::hash<int>()(c.x * 50 + c.y);
    }
};
struct node
{
    int x, y;
    coordinate from;
    double cost, heuristic;
    bool operator<(const node& a) const
    {
        return cost + heuristic > a.cost + a.heuristic;
    }
};


bool visited[maxLen][maxLen];
coordinate from[maxLen][maxLen];


enum ShipMode
{
    IDLE,
    ATTACK,
    REVENGE,
    HIDE,
    PRODUCE,
    CONSTRUCT,
    ROB,
    RUIN,
    FOLLOW
};


struct ShipInfo
{
    //THUAI7::Ship NearestEnemyShip;
    ShipMode mode;
    //THUAI7::Ship TargetShip;
    //coordinate TargetPos;
    //THUAI7::ConstructionType ConsType;
    //const char end = '\0';
};

ShipInfo myself = {IDLE};




namespace MapInfo
{
    enum Place
    {
        NullPlaceType = 0,
        Home = 1,
        Space = 2,
        Ruin = 3,
        Shadow = 4,
        Asteroid = 5,
        Resource = 6,
        Construction = 7,
        Wormhole = 8,
    };

    Place PlaceTypeConvert(THUAI7::PlaceType t)
    {
        switch (t)
        {
            case THUAI7::PlaceType::NullPlaceType:
                return NullPlaceType;
                break;
            case THUAI7::PlaceType::Home:
                return Home;
                break;
            case THUAI7::PlaceType::Space:
                return Space;
                break;
            case THUAI7::PlaceType::Ruin:
                return Ruin;
                break;
            case THUAI7::PlaceType::Shadow:
                return Shadow;
                break;
            case THUAI7::PlaceType::Asteroid:
                return Asteroid;
                break;
            case THUAI7::PlaceType::Resource:
                return Resource;
                break;
            case THUAI7::PlaceType::Construction:
                return Construction;
                break;
            case THUAI7::PlaceType::Wormhole:
                return Wormhole;
                break;
            default:
                break;
        }
    }

    Place fullmap[50][50];




    THUAI7::PlaceType map[maxLen][maxLen];

    std::unordered_set<coordinate, PointHash> des_list[4];
    coordinate home_pos, enemy_pos;
    bool hasGetMap = false;

    int getIndex(THUAI7::PlaceType type)
    {
        if (type == THUAI7::PlaceType::Resource)
            return 0;
        if (type == THUAI7::PlaceType::Construction)
            return 1;
        if (type == THUAI7::PlaceType::Wormhole)
            return 2;
    }

    void getMap(IShipAPI& api, bool top)
    {
        auto m = api.GetFullMap();
        for (int i = 0; i < m.size(); i++)
        {
            for (int j = 0; j < m[i].size(); j++)
            {
                auto t = m[i][j];
                map[i][j] = t;
                if (t == THUAI7::PlaceType::Resource or t == THUAI7::PlaceType::Construction or t == THUAI7::PlaceType::Wormhole)
                {
                    des_list[getIndex(m[i][j])].insert({i, j});
                }
                else if (t == THUAI7::PlaceType::Home)
                {
                    if (i < 25)
                    {
                        if (top)
                            MapInfo::home_pos = {i, j};
                        else
                            MapInfo::enemy_pos = {i, j};
                    }
                    else
                    {
                        if (!top)
                            MapInfo::home_pos = {i, j};
                        else
                            MapInfo::enemy_pos = {i, j};
                    }
                }
            }
        }
    }



    void LoadFullMap(ITeamAPI& api)
    {
        auto mp = api.GetFullMap();
        for (size_t i = 0; i < 50; i++)
        {
            for (size_t j = 0; j < 50; j++)
            {
                fullmap[i][j] = PlaceTypeConvert(mp[i][j]);
            }
        }
        
    }
}


namespace TeamBT
{

    /**
     * @enum NodeState
     * @brief 节点运行状态
     */
    enum NodeState
    {
        IDLE = 0,  ///< 未初始化值
        SUCCESS,   ///< 成功
        FAIL,      ///< 失败
        RUNNING    ///< 操作尚未完成，需要继续运行该节点
    };

    /**
     * @class rootNode
     * @brief 提供行为树节点的基本内容
     * @see NodeState
     */
    class rootNode
    {
    public:
        NodeState state;

        rootNode(NodeState x = IDLE) :
            state(x)
        {
        }

        virtual NodeState perform(ITeamAPI& api)
        {
            return IDLE;
        }

        virtual ~rootNode()
        {
        }
    };

    /**
     * @class eventNode
     * @brief 事件节点，当条件为真执行事件，并储存事件执行状态；否则节点状态为FAIL
     * @param condition 返回是否执行的布尔值的条件函数
     * @param action 待执行的操作对应的函数
     *
     */
    class eventNode : public rootNode
    {
    public:
        std::function<bool(ITeamAPI& api)> condition;    ///< 执行条件
        std::function<NodeState(ITeamAPI& api)> action;  ///< 待执行的函数
        eventNode(std::function<bool(ITeamAPI& api)> c, std::function<NodeState(ITeamAPI& api)> a) :
            condition(c),
            action(a)
        {
        }

        virtual NodeState perform(ITeamAPI& api)
        {
            switch (state)
            {
                case IDLE:
                    state = RUNNING;
                    break;
                case RUNNING:
                    break;
                default:
                    return state;
                    break;
            }

            if (condition(api))
            {
                state = action(api);
            }
            else
            {
                state = FAIL;
            }

            return state;
        }

        virtual ~eventNode()
        {
        }
    };

    /**
     * @class SequenceNode
     * @brief 队列节点，依次执行子节点，直到节点返回结果为FAIL或全部子节点都被执行完
     * @param state 与最后一个执行的子节点的状态相同
     */
    class SequenceNode : public rootNode
    {
    private:
        inline void RewindChildren()
        {
            for (size_t i = 0; i < events.size(); i++)
            {
                events[i]->state = IDLE;
            }
            curChild = 0;
        }

    public:
        std::vector<rootNode*> events;
        int curChild;

        /**
         * @brief 队列节点的perform函数\n
         * - 按照顺序执行子节点，如果成功，则下次调用时执行下一节点（如果全部执行完，将重置子节点状态）；如果失败，本队列节点的状态将被设为FAIL，并重置子节点状态
         * @param api
         * @return 仅当当前执行的子节点返回FAIL时返回FAIL、最后一个子节点返回SUCCESS时返回SUCCESS；否则返回RUNNING
         */
        virtual NodeState perform(ITeamAPI& api)
        {
            switch (state)
            {
                case IDLE:
                    state = RUNNING;
                    break;
                case RUNNING:
                    break;
                default:
                    return state;
                    break;
            }

            switch (events[curChild]->perform(api))
            {
                case RUNNING:
                    break;
                case SUCCESS:
                    if (curChild == events.size() - 1)
                    {
                        state = SUCCESS;
                        RewindChildren();
                    }
                    else
                    {
                        curChild++;
                    }
                    break;
                case FAIL:
                    state = FAIL;
                    RewindChildren();
                    break;
                default:
                    break;
            }
            return state;
        }
        SequenceNode(std::initializer_list<rootNode*> l) :
            events(l),
            curChild(0)
        {
        }

        SequenceNode(const SequenceNode& a) :
            curChild(0)
        {
            for (size_t i = 0; i < a.events.size(); i++)
            {
                events.push_back(new auto(*a.events[i]));
            }
        }

        virtual ~SequenceNode()
        {
            for (size_t i = 0; i < events.size(); i++)
            {
                delete events[i];
                events[i] = NULL;
            }
        }
    };

    /**
     * @brief 节点功能：反复执行子节点，直到返回SUCCESS
     *
     */
    class TryUntilSuccessNode : public rootNode
    {
    public:
        rootNode* child;
        TryUntilSuccessNode(rootNode* x) :
            rootNode(RUNNING),
            child(x)
        {
        }

        /**
         * @brief TryUntilSuccess对应的perform虚函数
         * @param api
         * @return 若子节点返回SUCCESS或当前状态已经为SUCCESS，则返回SUCCESS；反之返回RUNNING
         */
        virtual NodeState perform(ITeamAPI& api)
        {
            if (state == IDLE)
            {
                state = RUNNING;
            }
            else if (state == RUNNING)
            {
                switch (child->perform(api))
                {
                    case SUCCESS:
                        state = SUCCESS;
                        child->state = IDLE;
                        break;
                    case FAIL:
                        child->state = IDLE;
                        break;
                    default:
                        break;
                }
            }
            return state;
        }

        virtual ~TryUntilSuccessNode()
        {
            delete child;
            child = NULL;
        }
    };

    /**
     * @brief decorator节点，执行一次子节点，不论子节点返回值如何，均返回SUCCESS
     */
    class AlwaysSuccessNode : public rootNode
    {
    public:
        rootNode* child;  ///< 要执行的子节点
        AlwaysSuccessNode(rootNode* x) :
            rootNode(IDLE),
            child(x)
        {
        }

        virtual NodeState perform(ITeamAPI& api)
        {
            if (state == IDLE)
            {
                state = RUNNING;
            }
            if (state == RUNNING)
            {
                switch (child->perform(api))
                {
                    case RUNNING:
                        state = RUNNING;
                        break;
                    default:
                        state = SUCCESS;
                        break;
                }
            }
            return state;
        }

        virtual ~AlwaysSuccessNode()
        {
            delete child;
            child = NULL;
        }
    };

    /**
     * @brief 选择器节点，依次尝试每个子节点，直到有一个成功或全部失败
     */
    class fallbackNode : public rootNode
    {
    private:
        inline void RewindChildren()
        {
            for (size_t i = 0; i < events.size(); i++)
            {
                events[i]->state = IDLE;
            }
            curChild = 0;
        }

    public:
        std::vector<rootNode*> events;
        int curChild;

        /**
         * @brief 选择器节点的perform函数
         * - 如果当前子节点返回SUCCESS，则重置全部子节点状态，本节点状态设为SUCCESS并返回SUCCESS
         * - 如果当前子节点返回RUNNING，下一次调用时仍调用该子节点，直到它返回FAIL或SUCCESS；返回RUNNING
         * - 如果当前子节点返回FAIL
         *  . 如果已经是最后一个子节点，则本节点状态为FAIL，返回FAIL
         *  . 否则下一次调用执行下一个子节点，本次返回RUNNING
         * @param api
         * @return
         */
        virtual NodeState perform(ITeamAPI& api)
        {
            switch (state)
            {
                case IDLE:
                    state = RUNNING;
                    break;
                case RUNNING:
                    break;
                default:
                    return state;
                    break;
            }

            switch (events[curChild]->perform(api))
            {
                case RUNNING:
                    break;
                case FAIL:
                    if (curChild == events.size() - 1)
                    {
                        state = FAIL;
                        RewindChildren();
                    }
                    else
                    {
                        curChild++;
                    }
                    break;
                case SUCCESS:
                    state = SUCCESS;
                    RewindChildren();
                    break;
                default:
                    break;
            }
            return state;
        }

        fallbackNode(std::initializer_list<rootNode*> l) :
            events(l),
            curChild(0)
        {
        }

        fallbackNode(const fallbackNode& a) :
            curChild(0)
        {
            for (size_t i = 0; i < a.events.size(); i++)
            {
                events.push_back(new auto(*a.events[i]));
            }
        }

        virtual ~fallbackNode()
        {
            for (size_t i = 0; i < events.size(); i++)
            {
                delete events[i];
                events[i] = NULL;
            }
        }
    };

}

namespace ShipBT
{

    /**
     * @enum NodeState
     * @brief 节点运行状态
     */
    enum NodeState
    {
        IDLE = 0,  ///< 未初始化值
        SUCCESS,   ///< 成功
        FAIL,      ///< 失败
        RUNNING    ///< 操作尚未完成，需要继续运行该节点
    };

    /**
     * @class rootNode
     * @brief 提供行为树节点的基本内容
     * @see NodeState
     */
    class rootNode
    {
    public:
        NodeState state;

        rootNode(NodeState x = IDLE) :
            state(x)
        {
        }

        virtual NodeState perform(IShipAPI& api)
        {
            return IDLE;
        }

        virtual void Clear()
        {
            state = IDLE;
        }

        virtual ~rootNode()
        {
        }
    };

    /**
     * @class eventNode
     * @brief 事件节点，当条件为真执行事件，并储存事件执行状态；否则节点状态为FAIL
     * @param condition 返回是否执行的布尔值的条件函数
     * @param action 待执行的操作对应的函数
     *
     */
    class eventNode : public rootNode
    {
    public:
        std::function<bool(IShipAPI& api)> condition;    ///< 执行条件
        std::function<NodeState(IShipAPI& api)> action;  ///< 待执行的函数
        eventNode(std::function<bool(IShipAPI& api)> c, std::function<NodeState(IShipAPI& api)> a) :
            condition(c),
            action(a)
        {
        }

        virtual NodeState perform(IShipAPI& api)
        {
            switch (state)
            {
                case IDLE:
                    state = RUNNING;
                    break;
                case RUNNING:
                    break;
                default:
                    return state;
                    break;
            }

            if (condition(api))
            {
                state = action(api);
            }
            else
            {
                state = FAIL;
            }

            return state;
        }

        virtual void Clear()
        {
            state = IDLE;
        }

        virtual ~eventNode()
        {
        }
    };

    /**
     * @class SequenceNode
     * @brief 队列节点，依次执行子节点，直到节点返回结果为FAIL或全部子节点都被执行完
     * @param state 与最后一个执行的子节点的状态相同
     */
    class SequenceNode : public rootNode
    {
    private:
        inline void RewindChildren()
        {
            for (size_t i = 0; i < events.size(); i++)
            {
                events[i]->state = IDLE;
            }
            curChild = 0;
        }

    public:
        std::vector<rootNode*> events;
        int curChild;

        /**
         * @brief 队列节点的perform函数\n
         * - 按照顺序执行子节点，如果成功，则下次调用时执行下一节点（如果全部执行完，将重置子节点状态）；如果失败，本队列节点的状态将被设为FAIL，并重置子节点状态
         * @param api
         * @return 仅当当前执行的子节点返回FAIL时返回FAIL、最后一个子节点返回SUCCESS时返回SUCCESS；否则返回RUNNING
         */
        virtual NodeState perform(IShipAPI& api)
        {
            switch (state)
            {
                case IDLE:
                    state = RUNNING;
                    break;
                case RUNNING:
                    break;
                default:
                    return state;
                    break;
            }

            switch (events[curChild]->perform(api))
            {
                case RUNNING:
                    break;
                case SUCCESS:
                    if (curChild == events.size() - 1)
                    {
                        state = SUCCESS;
                        RewindChildren();
                    }
                    else
                    {
                        curChild++;
                    }
                    break;
                case FAIL:
                    state = FAIL;
                    RewindChildren();
                    break;
                default:
                    break;
            }
            return state;
        }

        virtual void Clear()
        {
            state = IDLE;
            for (size_t i = 0; i < events.size(); i++)
            {
                events[i]->Clear();
            }
        }

        SequenceNode(std::initializer_list<rootNode*> l) :
            events(l),
            curChild(0)
        {
        }

        SequenceNode(const SequenceNode& a) :
            curChild(0)
        {
            for (size_t i = 0; i < a.events.size(); i++)
            {
                events.push_back(new auto(*a.events[i]));
            }
        }

        virtual ~SequenceNode()
        {
            for (size_t i = 0; i < events.size(); i++)
            {
                delete events[i];
                events[i] = NULL;
            }
        }
    };

    /**
     * @brief 节点功能：反复执行子节点，直到返回SUCCESS
     *
     */
    class TryUntilSuccessNode : public rootNode
    {
    public:
        rootNode* child;
        TryUntilSuccessNode(rootNode* x) :
            rootNode(RUNNING),
            child(x)
        {
        }

        /**
         * @brief TryUntilSuccess对应的perform虚函数
         * @param api
         * @return 若子节点返回SUCCESS或当前状态已经为SUCCESS，则返回SUCCESS；反之返回RUNNING
         */
        virtual NodeState perform(IShipAPI& api)
        {
            if (state == IDLE)
            {
                state = RUNNING;
            }
            else if (state == RUNNING)
            {
                switch (child->perform(api))
                {
                    case SUCCESS:
                        state = SUCCESS;
                        child->state = IDLE;
                        break;
                    case FAIL:
                        child->state = IDLE;
                        break;
                    default:
                        break;
                }
            }
            return state;
        }

        virtual void Clear()
        {
            state = IDLE;
            child->Clear();
        }

        virtual ~TryUntilSuccessNode()
        {
            delete child;
            child = NULL;
        }
    };

    /**
     * @brief decorator节点，执行一次子节点，不论子节点返回值如何，均返回SUCCESS
     */
    class AlwaysSuccessNode : public rootNode
    {
    public:
        rootNode* child;  ///< 要执行的子节点
        AlwaysSuccessNode(rootNode* x) :
            rootNode(IDLE),
            child(x)
        {
        }

        virtual NodeState perform(IShipAPI& api)
        {
            if (state == IDLE)
            {
                state = RUNNING;
            }
            if (state == RUNNING)
            {
                switch (child->perform(api))
                {
                    case RUNNING:
                        state = RUNNING;
                        break;
                    default:
                        state = SUCCESS;
                        break;
                }
            }
            return state;
        }

        virtual void Clear()
        {
            state = IDLE;
            child->Clear();
        }

        virtual ~AlwaysSuccessNode()
        {
            delete child;
            child = NULL;
        }
    };

    /**
     * @brief 选择器节点，依次尝试每个子节点，直到有一个成功或全部失败
     */
    class fallbackNode : public rootNode
    {
    private:
        inline void RewindChildren()
        {
            for (size_t i = 0; i < events.size(); i++)
            {
                events[i]->state = IDLE;
            }
            curChild = 0;
        }

    public:
        std::vector<rootNode*> events;
        int curChild;

        /**
         * @brief 选择器节点的perform函数
         * - 如果当前子节点返回SUCCESS，则重置全部子节点状态，本节点状态设为SUCCESS并返回SUCCESS
         * - 如果当前子节点返回RUNNING，下一次调用时仍调用该子节点，直到它返回FAIL或SUCCESS；返回RUNNING
         * - 如果当前子节点返回FAIL
         *  . 如果已经是最后一个子节点，则本节点状态为FAIL，返回FAIL
         *  . 否则下一次调用执行下一个子节点，本次返回RUNNING
         * @param api
         * @return
         */
        virtual NodeState perform(IShipAPI& api)
        {
            switch (state)
            {
                case IDLE:
                    state = RUNNING;
                    break;
                case RUNNING:
                    break;
                default:
                    return state;
                    break;
            }

            switch (events[curChild]->perform(api))
            {
                case RUNNING:
                    break;
                case FAIL:
                    if (curChild == events.size() - 1)
                    {
                        state = FAIL;
                        RewindChildren();
                    }
                    else
                    {
                        curChild++;
                    }
                    break;
                case SUCCESS:
                    state = SUCCESS;
                    RewindChildren();
                    break;
                default:
                    break;
            }
            return state;
        }

        virtual void Clear()
        {
            state = IDLE;
            for (size_t i = 0; i < events.size(); i++)
            {
                events[i]->Clear();
            }
        }

        fallbackNode(std::initializer_list<rootNode*> l) :
            events(l),
            curChild(0)
        {
        }

        fallbackNode(const fallbackNode& a) :
            curChild(0)
        {
            for (size_t i = 0; i < a.events.size(); i++)
            {
                events.push_back(new auto(*a.events[i]));
            }
        }

        virtual ~fallbackNode()
        {
            for (size_t i = 0; i < events.size(); i++)
            {
                delete events[i];
                events[i] = NULL;
            }
        }
    };

}  // namespace ShipBT

namespace ShipFSM
{

    class State
    {
    public:
        std::function<bool(IShipAPI& api)> action;

        std::function<State*(IShipAPI& api)> transform;

        State* Invoke(IShipAPI& api)
        {
            action(api);
            return transform(api);
        }
    };

    class fsm
    {
    public:
        State * curstat;
        //std::vector<State*> stat_list;

        void Invoke(IShipAPI & api)
        {
            curstat = curstat->Invoke(api);
        }
    };



}

namespace Conditions
{
    bool always(ITeamAPI&)
    {
        return true;
    }
    auto EnergyThreshold(int threshold)
    {
        return [threshold](ITeamAPI& api)
        { return (api.GetEnergy() >= threshold); };
    }
}  // namespace Conditions

namespace HomeAction
{
    template<int id>
    auto SendMessage(const std::string& m)
    {
        return [=](ITeamAPI& api)
        {
            auto res = api.SendTextMessage(id, m);
            bool success = res.get();
            std::cout << "Send:" << success << std::endl;
            return success ? TeamBT::SUCCESS : TeamBT::FAIL;
        };
    }

    auto InstallModule(int id, THUAI7::ModuleType moduleType)
    {
        return [=](ITeamAPI& api)
        {
            auto res = api.InstallModule(id, moduleType);
            bool success = res.get();
            return success ? TeamBT::SUCCESS : TeamBT::FAIL;
        };
    }
    auto BuildShip(THUAI7::ShipType shipType)
    {
        return [=](ITeamAPI& api)
        {
            auto res = api.BuildShip(shipType, 0);
            bool success = res.get();
            return success ? TeamBT::SUCCESS : TeamBT::FAIL;
        };
    }
}

namespace Attack
{
    ShipFSM::State Wait;
    ShipFSM::State Shoot;
    ShipFSM::State Hide;
    ShipFSM::State Approach;








}


namespace revenge
{
    double WeaponToDis(THUAI7::WeaponType p)
    {
        switch (p)
        {
            case THUAI7::WeaponType::NullWeaponType:
                return 0;
                break;
            case THUAI7::WeaponType::LaserGun:
                return 4000;
                break;
            case THUAI7::WeaponType::PlasmaGun:
                return 4000;
                break;
            case THUAI7::WeaponType::ShellGun:
                return 4000;
                break;
            case THUAI7::WeaponType::MissileGun:
                return 8000;
                break;
            case THUAI7::WeaponType::ArcGun:
                return 8000;
                break;
            default:
                break;
        }
    }
    inline double min(double a, double b)
    {
        return a > b ? b : a;
    }

}

namespace Communication
{
    bool RefreshInfo(IShipAPI& api)
    {
        if (!api.HaveMessage())
        {
            return false;
        }
        else
        {

            std::string mes = (api.GetMessage()).second;
            memcpy(&myself, mes.data(), mes.size());


            std::cout << ((myself.mode == ATTACK) ? "yes" : "NO") << std::endl;
            return true;
        }
    }
}


//bool moved = false;


inline int manhatten_distance(int x1, int y1, int x2, int y2)
{
    return abs(x1 - x2) + abs(y1 - y2);
};
inline int manhatten_distance(int x1, int y1, coordinate c)
{
    return abs(x1 - c.x) + abs(y1 - c.y);
};
std::deque<coordinate> search_road(int x1, int y1, int x2, int y2)
{
    std::priority_queue<node> pq;
    pq.push({x1, y1, coordinate{-1, -1}, 0, (double)manhatten_distance(x1, y1, x2, y2)});
    memset(visited, false, sizeof(visited));
    while (!pq.empty())
    {
        node now = pq.top();
        pq.pop();
        if (visited[now.x][now.y])
            continue;
        visited[now.x][now.y] = true;
        from[now.x][now.y] = now.from;
        if (now.x == x2 && now.y == y2)
        {
            std::deque<coordinate> path;
            path.push_back({now.x, now.y});
            coordinate temp = now.from;
            while (temp.x != -1)
            {
                path.push_front({temp.x, temp.y});
                temp = from[temp.x][temp.y];
            }
            path.pop_front();
            return path;
        }
        auto is_empty = [](THUAI7::PlaceType t)
        {
            return t == THUAI7::PlaceType::Space or t == THUAI7::PlaceType::Shadow or t == THUAI7::PlaceType::Wormhole;
        };
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                if (i == 0 and j == 0)
                    continue;
                if (i == -1 and j == -1 and (!is_empty(MapInfo::map[now.x - 1][now.y]) or !is_empty(MapInfo::map[now.x][now.y - 1])))
                    continue;
                if (i == 1 and j == -1 and (!is_empty(MapInfo::map[now.x + 1][now.y]) or !is_empty(MapInfo::map[now.x][now.y - 1])))
                    continue;
                if (i == -1 and j == 1 and (!is_empty(MapInfo::map[now.x - 1][now.y]) or !is_empty(MapInfo::map[now.x][now.y + 1])))
                    continue;
                if (i == 1 and j == 1 and (!is_empty(MapInfo::map[now.x + 1][now.y]) or !is_empty(MapInfo::map[now.x][now.y + 1])))
                    continue;
                int nx = now.x + i, ny = now.y + j;
                if (nx < 0 or nx >= 50 or ny < 0 or ny >= 50 or visited[nx][ny])
                    continue;
                if (nx == x2 and ny == y2)
                {
                    double cost = sqrt(i * i + j * j);
                    pq.push({nx, ny, coordinate{now.x, now.y}, now.cost + cost, 0});
                }
                THUAI7::PlaceType t = MapInfo::map[nx][ny];
                if (t != THUAI7::PlaceType::Space and t != THUAI7::PlaceType::Shadow and t != THUAI7::PlaceType::Wormhole)
                    continue;
                double cost = sqrt(i * i + j * j);
                int h = manhatten_distance(nx, ny, x2, y2);
                pq.push({nx, ny, coordinate{now.x, now.y}, now.cost + cost, (double)h});
            }
        }
    }
}
std::deque<coordinate> search_road(int x1, int y1, THUAI7::PlaceType type, IShipAPI& api)
{
    int min_dis = 0x7fffffff, index = MapInfo::getIndex(type);
    auto& des = MapInfo::des_list[index];
    for (auto i = des.begin(); i != des.end();)
    {
        if (type == THUAI7::PlaceType::Resource)
        {
            std::cout << "x: " << (*i).x << " y: " << (*i).y << std::endl;
            int temp = api.GetResourceState((*i).x, (*i).y);
            if (temp == 0)
            {
                i = des.erase(i);
                continue;
            }
        }
        else if (type == THUAI7::PlaceType::Construction and api.GetConstructionHp((*i).x, (*i).y) > 0)
        {
            i = des.erase(i);
            continue;
        }
        int temp = manhatten_distance(x1, y1, *i);
        if (temp < min_dis)
            min_dis = temp;
        i++;
    }
    std::priority_queue<node> pq;
    pq.push({x1, y1, coordinate{-1, -1}, 0, (double)min_dis});
    memset(visited, false, sizeof(visited));
    while (!pq.empty())
    {
        node now = pq.top();
        pq.pop();
        if (visited[now.x][now.y])
            continue;
        visited[now.x][now.y] = true;
        from[now.x][now.y] = now.from;
        if (des.find({now.x, now.y}) != des.end())
        {
            std::deque<coordinate> path;
            path.push_back({now.x, now.y});
            coordinate temp = now.from;
            while (temp.x != -1)
            {
                path.push_front({temp.x, temp.y});
                temp = from[temp.x][temp.y];
            }
            path.pop_front();
            return path;
        }
        auto is_empty = [](THUAI7::PlaceType t)
        {
            return t == THUAI7::PlaceType::Space or t == THUAI7::PlaceType::Shadow or t == THUAI7::PlaceType::Wormhole;
        };
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                if (i == 0 and j == 0)
                    continue;
                if (i == -1 and j == -1 and (!is_empty(MapInfo::map[now.x - 1][now.y]) or !is_empty(MapInfo::map[now.x][now.y - 1])))
                    continue;
                if (i == 1 and j == -1 and (!is_empty(MapInfo::map[now.x + 1][now.y]) or !is_empty(MapInfo::map[now.x][now.y - 1])))
                    continue;
                if (i == -1 and j == 1 and (!is_empty(MapInfo::map[now.x - 1][now.y]) or !is_empty(MapInfo::map[now.x][now.y + 1])))
                    continue;
                if (i == 1 and j == 1 and (!is_empty(MapInfo::map[now.x + 1][now.y]) or !is_empty(MapInfo::map[now.x][now.y + 1])))
                    continue;
                int nx = now.x + i, ny = now.y + j;
                if (nx < 0 or nx >= 50 or ny < 0 or ny >= 50 or visited[nx][ny])
                    continue;
                if (des.find({nx, ny}) != des.end() and (i == 0 or j == 0))
                {
                    double cost = sqrt(i * i + j * j);
                    pq.push({nx, ny, coordinate{now.x, now.y}, now.cost + cost, 0});
                }
                THUAI7::PlaceType t = MapInfo::map[nx][ny];
                if (t != THUAI7::PlaceType::Space and t != THUAI7::PlaceType::Shadow and t != THUAI7::PlaceType::Wormhole)
                    continue;
                double cost = sqrt(i * i + j * j);
                min_dis = 0x7fffffff;
                for (auto const& k : des)
                {
                    int temp = manhatten_distance(x1, y1, k);
                    if (temp < min_dis)
                        min_dis = temp;
                }
                pq.push({nx, ny, coordinate{now.x, now.y}, now.cost + cost, (double)min_dis});
            }
        }
    }
}

//coordinate GetNearestResource(int x1, int y1, THUAI7::PlaceType type, IShipAPI& api)
//{
//    int min_dis = 0x7fffffff, index = MapInfo::getIndex(type);
//    auto& des = MapInfo::des_list[index];
//    auto i = des.begin();
//    for (; i != des.end();)
//    {
//        if (type == THUAI7::PlaceType::Resource)
//        {
//            std::cout << "x: " << (*i).x << " y: " << (*i).y << std::endl;
//            int temp = api.GetResourceState((*i).x, (*i).y);
//            if (temp == 0)
//            {
//                i = des.erase(i);
//                continue;
//            }
//        }
//        else if (type == THUAI7::PlaceType::Construction and api.GetConstructionHp((*i).x, (*i).y) > 0)
//        {
//            i = des.erase(i);
//            continue;
//        }
//        int temp = manhatten_distance(x1, y1, *i);
//        if (temp < min_dis)
//            min_dis = temp;
//        i++;
//    }
//    return *i;
//}

std::deque<coordinate> path;
coordinate target_pos;

enum class shipState
{
    produce,
    construct,
    back,
};
shipState this_state;  //= shipState::produce;
THUAI7::ConstructionType construction_type;
bool judgeNear(int x, int y, THUAI7::PlaceType t)
{
    x = x / 1000, y = y / 1000;
    if (MapInfo::map[x - 1][y] == t or MapInfo::map[x][y - 1] == t or MapInfo::map[x + 1][y] == t or MapInfo::map[x][y + 1] == t)
    {
        return true;
    }
    return false;
};

//class MyPath
//{
//public:
//    enum PathState
//    {
//        IDLE,
//        RUNNING,
//        ARRIVED
//    };
//
//    std::deque<coordinate> path;
//    coordinate destination;
//    coordinate curpos;
//    PathState mystate;
//
//
//
//    MyPath() :
//        mystate(IDLE)
//    {
//    }
//    
//    MyPath& SetDes(coordinate d)
//    {
//        path.clear();
//        destination = d;
//        mystate = IDLE;
//        return *this;
//    }
//
//    PathState Move(IShipAPI& api)
//    {
//        auto inf = api.GetSelfInfo();
//        if (mystate==IDLE)
//        {
//            mystate = RUNNING;
//            curpos.x = inf->x, curpos.y = inf->y;
//            path = search_road(curpos.x, curpos.y, destination.x, destination.y);
//        }
//        else if (curpos.x!=inf->x || curpos.y!=inf->y)
//        {
//            curpos.x = inf->x, curpos.y = inf->y;
//            path = search_road(curpos.x, curpos.y, destination.x, destination.y);
//        }
//        else if (mystate==ARRIVED)
//        {
//            return ARRIVED;
//        }
//        
//
//        auto next = path.front();
//        
//        int dx = next.x * 1000 + 500 - curpos.x;
//        int dy = next.y * 1000 + 500 - curpos.y;
//        std::cout << "x: " << curpos.x << " y: " << curpos.y << "nx: " << next.x << "ny: " << next.y << std::endl;
//        std::cout << "dx: " << dx << " dy: " << dy << std::endl;
//        double time = sqrt(dx * dx + dy * dy) / 3.0;
//        double angle = atan2(dy, dx);
//        std::cout << "time: " << time << " angle: " << angle << std::endl;
//        if (inf->shipState!=THUAI7::ShipState::Idle)
//        {
//            api.EndAllAction();
//        }
//        if (api.Move(time, angle).get())
//        {
//            path.pop_front();
//            curpos.x = next.x * 1000;
//            curpos.y = next.y * 1000;
//        }
//        
//        if (abs(curpos.x-destination.x)<=1000&&abs(curpos.y-destination.y)<=1000)
//        {
//            mystate = ARRIVED;
//        }
//
//        return mystate;
//    }
//
//    bool arrived()
//    {
//        return (mystate == ARRIVED) ? true : false;
//    }
//
//};
//
//MyPath CivilPath;


void civilShip(IShipAPI& api)
{
    auto info = api.GetSelfInfo();
    int x = info->x, y = info->y;
    if (this_state == shipState::produce)
    {
        if (info->shipState == THUAI7::ShipState::Producing)
            return;
        if (path.empty())
        {
            if (judgeNear(x, y, THUAI7::PlaceType::Resource))
            {
                int left = api.GetResourceState(target_pos.x, target_pos.y);
                // std::cout << "distance: " << left << std::endl;
                auto res = api.Produce();
                bool success = res.get();
                std::cout << (success ? "success" : "failed") << std::endl;
                if (!success)
                {
                    path = search_road(x / 1000, y / 1000, THUAI7::PlaceType::Resource, api);
                }
            }
            else
            {
                path = search_road(x / 1000, y / 1000, THUAI7::PlaceType::Resource, api);
                target_pos = path.back();
            }
        }
    }
    else if (this_state == shipState::construct)
    {
        if (info->shipState == THUAI7::ShipState::Constructing)
            return;
        if (path.empty())
        {
            if (judgeNear(x, y, THUAI7::PlaceType::Construction))
            {
                api.Construct(construction_type);
            }
            else
            {
                path = search_road(x / 1000, y / 1000, THUAI7::PlaceType::Construction, api);
                target_pos = path.back();
            }
        }
    }
    else if (this_state == shipState::back)
    {
        if (path.empty())
        {
            path = search_road(x / 1000, y / 1000, MapInfo::home_pos.x, MapInfo::home_pos.y);
            api.EndAllAction();
        }
    }
    if (api.GetSelfInfo()->shipState == THUAI7::ShipState::Idle and !path.empty())
    {
        auto next = path.front();
        path.pop_front();
        int dx = next.x * 1000 + 500 - x;
        int dy = next.y * 1000 + 500 - y;
        std::cout << "x: " << x << " y: " << y << "nx: " << next.x << "ny: " << next.y << std::endl;
        std::cout << "dx: " << dx << " dy: " << dy << std::endl;
        double time = sqrt(dx * dx + dy * dy) / 3.0;
        double angle = atan2(dy, dx);
        std::cout << "time: " << time << " angle: " << angle << std::endl;
        api.Move(time, angle);
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

#include<iomanip>

//void AI::play(IShipAPI& api)
//{
//    auto info = api.GetSelfInfo();
//    int x = info->x, y = info->y;
//
//
//    if (!MapInfo::hasGetMap)
//    {
//        MapInfo::hasGetMap = true;
//        MapInfo::getMap(api, x / 1000 < 25 ? true : false);
//    }
//
//
//
//    //std::string mes = api.GetMessage().second;
//
//    //for (size_t i = 0; i < mes.size(); i++)
//    //{
//    //    std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(mes[i]) << " ";
//    //}
//    //std::cout << std::endl;
//
//    if (api.HaveMessage())
//    {
//        auto m = api.GetMessage();
//        std::string message = m.second;
//        if (message == "produce")
//        {
//            this_state = shipState::produce;
//        }
//        else if (message.find("construct") == 0)
//        {
//            this_state = shipState::construct;
//            auto type = message.substr(9);
//            if (type == "factory")
//            {
//                construction_type = THUAI7::ConstructionType::Factory;
//            }
//            else if (type == "fort")
//            {
//                construction_type = THUAI7::ConstructionType::Fort;
//            }
//            else if (type == "community")
//            {
//                construction_type = THUAI7::ConstructionType::Community;
//            }
//        }
//        else if (message.find("back") == 0)
//        {
//            this_state = shipState::back;
//            std::cout << "back" << std::endl;
//        }
//    }
//    if (this->playerID == 1)
//    {
//
//
//
//
//        civilShip(api);
//    }
//    else if (this->playerID == 2)
//    {
//        civilShip(api);
//    }
//
//    else if (this->playerID == 3)
//    {
//        auto info = api.GetSelfInfo();
//        int x = info->x, y = info->y;
//        if (path.empty())
//        {
//            path = search_road(x / 1000, y / 1000, THUAI7::PlaceType::Wormhole, api);
//        }
//        if (api.GetSelfInfo()->shipState == THUAI7::ShipState::Idle and !path.empty())
//        {
//            auto next = path.front();
//            path.pop_front();
//            int dx = next.x * 1000 + 500 - x;
//            int dy = next.y * 1000 + 500 - y;
//            std::cout << "x: " << x << " y: " << y << "nx: " << next.x << "ny: " << next.y << std::endl;
//            std::cout << "dx: " << dx << " dy: " << dy << std::endl;
//            double time = sqrt(dx * dx + dy * dy) / 3.0;
//            double angle = atan2(dy, dx);
//            std::cout << "time: " << time << " angle: " << angle << std::endl;
//            api.Move(time, angle);
//        }
//    }
//    else if (this->playerID == 4)
//    {
//        api.MoveDown(100);
//        std::this_thread::sleep_for(std::chrono::seconds(1));
//        api.MoveLeft(100);
//    }
//}

//std::string operator==(ShipInfo a, ShipInfo b)
//{
//    if (a.ConsType==b.ConsType&&a.mode==b.mode)
//    {
//        return "yes";
//    }
//    return "fail";
//}


ShipMode cur_Mode = IDLE;




ShipBT::rootNode* shipTreeList[9];


void AI::play(IShipAPI& api)
{
    Communication::RefreshInfo(api);



     //if (myself.mode!=cur_Mode)
     //{
     //    shipTreeList[cur_Mode]->Clear();
     //    cur_Mode = myself.mode;

     //}

     //shipTreeList[cur_Mode]->perform(api);
}








//const char* get_placetype(THUAI7::PlaceType t);
bool hasSend = false;
bool hasInstall = false;
bool hasBuild = false;
bool BuildSecondCivil = false;






volatile int Ind = 2;

TeamBT::SequenceNode root = {
    new TeamBT::AlwaysSuccessNode(new TeamBT::eventNode({Conditions::always, HomeAction::SendMessage<1>("produce")})),
    new TeamBT::TryUntilSuccessNode(new TeamBT::eventNode({Conditions::EnergyThreshold(8000), HomeAction::InstallModule(1, THUAI7::ModuleType::ModuleProducer3)})),
    new TeamBT::TryUntilSuccessNode(new TeamBT::eventNode({Conditions::EnergyThreshold(4000), HomeAction::BuildShip(THUAI7::ShipType::CivilianShip)})),
    new TeamBT::AlwaysSuccessNode(new TeamBT::eventNode({Conditions::always, HomeAction::SendMessage<2>("constructfactory")})),
    new TeamBT::TryUntilSuccessNode(new TeamBT::eventNode({Conditions::EnergyThreshold(12000), HomeAction::BuildShip(THUAI7::ShipType::MilitaryShip)})),
    new TeamBT::TryUntilSuccessNode(new TeamBT::eventNode({Conditions::EnergyThreshold(8000), HomeAction::InstallModule(2, THUAI7::ModuleType::ModuleProducer3)})),
    new TeamBT::TryUntilSuccessNode(new TeamBT::eventNode({Conditions::EnergyThreshold(10000), HomeAction::InstallModule(1, THUAI7::ModuleType::ModuleLaserGun)}))
};



ShipInfo ships[4];


const size_t SHIPINFO_SIZE = sizeof(ShipInfo);
using bytePointer = unsigned char*;

void sync_ships(ITeamAPI& api)
{
    auto players = api.GetShips();
    for (size_t i = 0; i < players.size(); i++)
    {
        int ind = players[i]->playerID;
        bytePointer pos = (bytePointer)&ships[i];
        std::string message;
        message.resize(SHIPINFO_SIZE + 1);
        memcpy(message.data(), &ships[i], SHIPINFO_SIZE);
        message[SHIPINFO_SIZE] = '\0';
        api.SendBinaryMessage(ind, message);
    }
}



bool run = true;

const char* get__placetype(THUAI7::PlaceType t);


void AI::play(ITeamAPI& api)  // 默认team playerID 为0
{
    //root.perform(api);
    ships[0] = {
        ATTACK
    };
    sync_ships(api);
    //api.SendTextMessage(1, "hello");
    //api.SendTextMessage(2, "hello");
    //Ind = 1;
    //if (run)
    //{
    //    if (root.perform(api)!=TeamBT::RUNNING)
    //    {
    //        run = false;
    //    }
    //}
    
    //auto mp = api.GetFullMap();

    //for (size_t i = 0; i < mp.size(); i++)
    //{
    //    for (size_t j = 0; j < mp[0].size(); j++)
    //    {
    //        std::cout << get__placetype(mp[i][j]) << " ";
    //    }
    //    std::cout << "\n";
    //}

    //std::cout << api.GetSelfInfo()->teamID << "  "<<api.GetEnergy() << std::endl;
}


        const char* get__placetype(THUAI7::PlaceType t)
{
    switch (t)
    {
        case THUAI7::PlaceType::NullPlaceType:
            return "nullplacetype";
        case THUAI7::PlaceType::Home:
            return "home";
        case THUAI7::PlaceType::Space:
            return "space";
        case THUAI7::PlaceType::Ruin:
            return "ruin";
            break;
        case THUAI7::PlaceType::Shadow:
            return "shadow";
            break;
        case THUAI7::PlaceType::Asteroid:
            return "asteroid";
        case THUAI7::PlaceType::Resource:
            return "resource";
        case THUAI7::PlaceType::Construction:
            return "construction";
            break;
        case THUAI7::PlaceType::Wormhole:
            return "wormhole";
        default:
            break;
    }
    return "?";
}

        /*
        if (!moved)
        {
             std::this_thread::sleep_for(std::chrono::seconds(6));
        double time = (y - 41.5 * 1000) / 3.0;
        api.MoveLeft(time);
        std::cout << time << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(int(time)));
        info = api.GetSelfInfo();
        std::cout << "x: " << info->x << " y: " << info->y << std::endl;
        time = (14 * 1000 - x) / 3.0;
        api.MoveDown(time);
        std::this_thread::sleep_for(std::chrono::milliseconds(int(time)+5));
        info = api.GetSelfInfo();
        std::cout << "x: " << info->x << " y: " << info->y << std::endl;
        time = 1000.0;
        api.MoveRight(time);
        std::this_thread::sleep_for(std::chrono::milliseconds(int(time)));
        info = api.GetSelfInfo();
        std::cout << "x: " << info->x << " y: " << info->y << std::endl;
        api.Produce();
        moved = true;
        }*/
