/*
Copyright 2021, Yang Luo"
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

@Author
Yang Luo, PHD
@email: yluo@hit.edu.cn

@Created on: 2021.11.29
@Last Modified: 2023.3.28 22:03
*/


/*-----------------------------------------------------------------------------
 * ecat_type.h
 * Description              EtherCAT Type Definitions
 *
 *---------------------------------------------------------------------------*/

#ifndef ECAT_TYPE_H
#define ECAT_TYPE_H


#include <semaphore.h> //sem
#include <cinttypes>

#define MAX_SLAVE_NUM 50     // Maximal number of slaves in the EtherCAT network
#define MAX_PDINPUT_NUM 25   // Maximal number of PD Inputs per slave
#define MAX_PDOUTPUT_NUM 25  // Maximal number of PD Outputs per slave

#define MAX_PD_NAME_LEN 72    // Maximal length of a PD Variable name
#define MAX_SLAVE_NAME_LEN 80 // Maximal length of a slave name

#define EC_SEM_MUTEX "sync"
#define EC_SEM_NUM 10

#define EC_SHM "ecm"
#define EC_SHM_MAX_SIZE 5242880 // 5MB


#define ECAT_STATE_INIT 1
#define ECAT_STATE_PREOP 2
#define ECAT_STATE_SAFEOP 4
#define ECAT_STATE_OP 8
#define ECAT_STATE_BOOTSTRAP 3


namespace rocos {
    struct PdVar {
        char name[MAX_PD_NAME_LEN]      {'\0'};
        int  offset                     {-1};
        int  size                       {-1};
        uint16_t index                {0};
        uint8_t  sub_index             {0};
    };

    struct Slave {
        int id                          {-1};
        char name[MAX_SLAVE_NAME_LEN]   {'\0'};;

        int input_var_num               {0};
        int output_var_num              {0};

        PdVar input_vars[MAX_PDINPUT_NUM];
        PdVar output_vars[MAX_PDOUTPUT_NUM];
    };

    struct EcatBus {
        long timestamp               {0};

        double min_cycle_time        {0.0};
        double max_cycle_time        {0.0};
        double avg_cycle_time        {0.0};
        double current_cycle_time    {0.0};

        bool   resetCycleTime        {false};

        int current_state            {ECAT_STATE_INIT};
        int request_state            {ECAT_STATE_OP};
        int next_expected_state      {}; // internal use by think

        bool is_authorized           {false};

        int slave_num                 {0};
        Slave slaves[MAX_SLAVE_NUM];

    };

}



#endif //ROCOS_ECM_ECAT_TYPE_H
