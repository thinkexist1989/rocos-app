// Copyright 2026, Yang Luo
// SPDX-License-Identifier: GPL-3.0-or-later
//
// shared_memory_config.hpp
// Single header-only class that covers both the EtherCAT master side
// (create/own shared memory) and the client side (open/read shared memory).
//
// Backward-compatible aliases are provided at the bottom:
//   rocos::EcatConfigMaster  ->  rocos::SharedMemoryConfig
//   rocos::EcatConfig        ->  rocos::SharedMemoryConfig

#pragma once

#include <algorithm>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <limits>
#include <map>
#include <semaphore.h>
#include <sstream>
#include <string>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <thread>
#include <mutex>
#include <stdexcept>
#include <unistd.h>
#include <vector>

// --------------------------------------------------------------------------
// EtherCAT shared-memory constants
// --------------------------------------------------------------------------
#ifndef EC_SHM
#  define EC_SHM          "ecm"
#endif
#ifndef EC_SEM_MUTEX
#  define EC_SEM_MUTEX    "sync"
#endif
#ifndef EC_SEM_NUM
#  define EC_SEM_NUM      10
#endif
#ifndef EC_SHM_MAX_SIZE
#  define EC_SHM_MAX_SIZE 5242880  // 5 MB
#endif

#ifndef MAX_SLAVE_NUM
#  define MAX_SLAVE_NUM      50
#endif
#ifndef MAX_PDINPUT_NUM
#  define MAX_PDINPUT_NUM    25
#endif
#ifndef MAX_PDOUTPUT_NUM
#  define MAX_PDOUTPUT_NUM   25
#endif
#ifndef MAX_PD_NAME_LEN
#  define MAX_PD_NAME_LEN    72
#endif
#ifndef MAX_SLAVE_NAME_LEN
#  define MAX_SLAVE_NAME_LEN 80
#endif

#ifndef ECAT_STATE_INIT
#  define ECAT_STATE_INIT      1
#  define ECAT_STATE_PREOP     2
#  define ECAT_STATE_SAFEOP    4
#  define ECAT_STATE_OP        8
#  define ECAT_STATE_BOOTSTRAP 3
#endif

namespace rocos {

// --------------------------------------------------------------------------
// Data structures shared between master and client
// --------------------------------------------------------------------------
/**
 * @brief Cross-process descriptor of a single process-data variable.
 */
struct PdVar {
    char     name[MAX_PD_NAME_LEN]{'\0'}; ///< Variable name (null-terminated).
    int      offset{-1};                  ///< Byte offset into the PDO buffer.
    int      size{-1};                    ///< Size in bytes (must equal sizeof(T)).
    uint16_t index{0};                    ///< PDO object index.
    uint8_t  sub_index{0};                ///< PDO object sub-index.
};

/**
 * @brief Cross-process descriptor of one slave and its PDO variables.
 */
struct Slave {
    int  id{-1};                          ///< Slave index.
    char name[MAX_SLAVE_NAME_LEN]{'\0'};  ///< Slave name (null-terminated).

    int input_var_num{0};                 ///< Number of valid input_vars entries.
    int output_var_num{0};                ///< Number of valid output_vars entries.

    PdVar input_vars[MAX_PDINPUT_NUM];    ///< Input (TxPDO) variables.
    PdVar output_vars[MAX_PDOUTPUT_NUM];  ///< Output (RxPDO) variables.
};

/**
 * @brief Cross-process bus snapshot shared between master and clients.
 *
 * This is an ABI type: do not change field order, types, or array sizes.
 */
struct EcatBus {
    long   timestamp{0}; ///< Last cycle monotonic timestamp [us].

    double min_cycle_time{0.0};      ///< Shortest observed cycle [us].
    double max_cycle_time{0.0};      ///< Longest observed cycle [us].
    double avg_cycle_time{0.0};      ///< Running average cycle [us].
    double current_cycle_time{0.0};  ///< Most recent cycle [us].

    uint32_t dt{0}; ///< Control cycle period [microseconds], e.g. 1000 = 1 kHz.
    
    bool   resetCycleTime{false}; ///< Client request to reset cycle statistics.

    int    current_state{ECAT_STATE_INIT}; ///< Current bus state.
    int    request_state{ECAT_STATE_OP};   ///< Client-requested state.
    int    next_expected_state{};          ///< Internal use only.

    bool   is_authorized{false}; ///< True when link and working counters are healthy.

    int    slave_num{0};          ///< Number of valid slaves entries.
    Slave  slaves[MAX_SLAVE_NUM]; ///< Slave descriptors.
};

// --------------------------------------------------------------------------
// SharedMemoryConfig
//
// Master usage (direct instantiation):
//   SharedMemoryConfig master(id);
//   master.createSharedMemory();
//   master.createPdDataMemoryProvider(inSz, outSz);
//
// Client usage (singleton, auto-connects on first call):
//   SharedMemoryConfig *cfg = SharedMemoryConfig::getInstance(id);
// --------------------------------------------------------------------------
/**
 * @brief Master/client owner of the shared-memory and semaphore IPC objects.
 *
 * The master side instantiates directly and calls createSharedMemory() plus
 * createPdDataMemoryProvider(). The client side uses getInstance() to
 * auto-connect on first use.
 */
class SharedMemoryConfig {
public:
    // -----------------------------------------------------------------------
    // Construction / Destruction
    // -----------------------------------------------------------------------
    /**
     * @brief Constructs for a non-negative master id.
     * @param id Master id; throws std::invalid_argument when negative.
     */
    explicit SharedMemoryConfig(int id = 0) {
        if (id < 0) {
            throw std::invalid_argument("SharedMemoryConfig requires a non-negative master id");
        }
        ecmName     = EC_SHM      + std::to_string(id);
        mutexName   = EC_SEM_MUTEX + std::to_string(id) + "_";
        pdInputName  = "pd_input"  + std::to_string(id);
        pdOutputName = "pd_output" + std::to_string(id);
    }

    /// @brief Releases mappings, closes descriptors, and unlinks owned resources.
    ~SharedMemoryConfig() {
        releaseEcm();
        releasePdInput();
        releasePdOutput();
        closeSemaphores();
        unlinkOwnedResources();
    }

    // Non-copyable, non-movable (owns OS resources)
    SharedMemoryConfig(const SharedMemoryConfig &)            = delete;
    SharedMemoryConfig &operator=(const SharedMemoryConfig &) = delete;

    // -----------------------------------------------------------------------
    // Client singleton factory
    // Automatically calls init() (getSharedMemory + getPdDataMemoryProvider).
    // -----------------------------------------------------------------------
    /**
     * @brief Returns the per-id client singleton, auto-connecting on first call.
     * @param id Master id to connect to.
     * @return The singleton instance (never null).
     */
    static SharedMemoryConfig *getInstance(int id = 0) {
        static std::map<int, SharedMemoryConfig *> instances;
        static std::mutex inst_mutex;
        std::lock_guard<std::mutex> lk(inst_mutex);
        if (instances.find(id) == instances.end()) {
            std::cout << "[SHM] Create new SharedMemoryConfig instance: " << id << std::endl;
            instances[id] = new SharedMemoryConfig(id);
            instances[id]->init();
        }
        return instances[id];
    }

    // -----------------------------------------------------------------------
    // Master-side: create (and own) shared memory objects
    // -----------------------------------------------------------------------
    /**
     * @brief Creates and maps the `ecm{id}` bus shared memory and its semaphores.
     * @return True when all objects are created and mapped.
     */
    bool createSharedMemory() {
        mode_t mask = umask(0);

        rollbackOwnedEcm();
        rollbackOwnedSemaphores();

        const std::string shm_name = toPosixName(ecmName);

        // Do not O_TRUNC yet: an existing object must survive until the
        // exclusive flock below proves no live owner still holds it.
        ecm_fd_ = shm_open(shm_name.c_str(), O_RDWR | O_CREAT, 0660);
        if (ecm_fd_ < 0) {
            print_message("[SHM] Cannot create " + shm_name + ": " + std::strerror(errno), MessageLevel::ERROR);
            umask(mask); return false;
        }
        // flock is held for as long as this fd stays open, so it ties
        // ownership to process lifetime: a live master keeps the lock, and
        // this call fails fast instead of silently truncating its memory. A
        // stale object left by a crashed process has no lock holder, so the
        // lock is acquired immediately and the object is safely reclaimed.
        if (flock(ecm_fd_, LOCK_EX | LOCK_NB) != 0) {
            if (errno == EWOULDBLOCK) {
                print_message("[SHM] " + shm_name + " is owned by a running master (master id already active)", MessageLevel::ERROR);
            } else {
                print_message("[SHM] Cannot lock " + shm_name + ": " + std::strerror(errno), MessageLevel::ERROR);
            }
            close(ecm_fd_);
            ecm_fd_ = -1;
            umask(mask); return false;
        }
        owns_ecm_ = true;
        if (ftruncate(ecm_fd_, static_cast<off_t>(ecm_size_)) != 0) {
            print_message("[SHM] Cannot resize " + shm_name + ": " + std::strerror(errno), MessageLevel::ERROR);
            rollbackOwnedEcm();
            umask(mask); return false;
        }
        void *addr = mmap(nullptr, ecm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, ecm_fd_, 0);
        if (addr == MAP_FAILED) {
            print_message("[SHM] Cannot map " + shm_name + ": " + std::strerror(errno), MessageLevel::ERROR);
            rollbackOwnedEcm();
            umask(mask); return false;
        }
        ecatBus  = static_cast<EcatBus *>(addr);
        *ecatBus = EcatBus{};  // zero-initialise

        for (int i = 0; i < EC_SEM_NUM; i++) {
            const std::string semName = semaphoreName(i);
            // Unlink any existing semaphore so sem_open always creates a fresh
            // one with value 0, even after a crash that skipped the destructor.
            (void)sem_unlink(semName.c_str());
            sem_mutex[i] = sem_open(semName.c_str(), O_CREAT | O_EXCL, 0660, 0);
            if (sem_mutex[i] == SEM_FAILED) {
                print_message("[SHM] Cannot create semaphore " + semName + ": " + std::strerror(errno), MessageLevel::ERROR);
                rollbackCreatedSemaphores(i);
                rollbackOwnedEcm();
                umask(mask); return false;
            }
        }
        owns_semaphores_ = true;

        umask(mask);
        return true;
    }

    /**
     * @brief Creates and maps the `pd_input{id}` and `pd_output{id}` PDO buffers.
     * @param pdInputSize  Input PDO buffer size in bytes.
     * @param pdOutputSize Output PDO buffer size in bytes.
     * @return True on success.
     */
    bool createPdDataMemoryProvider(int pdInputSize, int pdOutputSize) {
        if (!isValidRegionSize(pdInputSize) || !isValidRegionSize(pdOutputSize)) {
            return false;
        }

        rollbackOwnedPdInput();
        rollbackOwnedPdOutput();

        const std::string pd_in  = toPosixName(pdInputName);
        const std::string pd_out = toPosixName(pdOutputName);

        // Same reclaim-vs-reject logic as createSharedMemory(): no O_TRUNC
        // until the exclusive flock proves no live owner holds this object.
        pd_input_fd_ = shm_open(pd_in.c_str(), O_RDWR | O_CREAT, 0660);
        if (pd_input_fd_ < 0) {
            print_message("[SHM] Cannot create " + pd_in + ": " + std::strerror(errno), MessageLevel::ERROR);
            return false;
        }
        if (flock(pd_input_fd_, LOCK_EX | LOCK_NB) != 0) {
            if (errno == EWOULDBLOCK) {
                print_message("[SHM] " + pd_in + " is owned by a running master (master id already active)", MessageLevel::ERROR);
            } else {
                print_message("[SHM] Cannot lock " + pd_in + ": " + std::strerror(errno), MessageLevel::ERROR);
            }
            close(pd_input_fd_);
            pd_input_fd_ = -1;
            return false;
        }
        owns_pd_input_ = true;
        if (ftruncate(pd_input_fd_, static_cast<off_t>(pdInputSize)) != 0) {
            print_message("[SHM] Cannot resize " + pd_in + ": " + std::strerror(errno), MessageLevel::ERROR);
            rollbackOwnedPdInput();
            return false;
        }
        pdInputPtr = mmap(nullptr, static_cast<std::size_t>(pdInputSize), PROT_READ | PROT_WRITE, MAP_SHARED, pd_input_fd_, 0);
        if (pdInputPtr == MAP_FAILED) {
            print_message("[SHM] Cannot map " + pd_in + ": " + std::strerror(errno), MessageLevel::ERROR);
            pdInputPtr = nullptr;
            rollbackOwnedPdInput();
            return false;
        }
        pd_input_size_ = static_cast<std::size_t>(pdInputSize);

        pd_output_fd_ = shm_open(pd_out.c_str(), O_RDWR | O_CREAT, 0660);
        if (pd_output_fd_ < 0) {
            print_message("[SHM] Cannot create " + pd_out + ": " + std::strerror(errno), MessageLevel::ERROR);
            rollbackOwnedPdInput();
            return false;
        }
        if (flock(pd_output_fd_, LOCK_EX | LOCK_NB) != 0) {
            if (errno == EWOULDBLOCK) {
                print_message("[SHM] " + pd_out + " is owned by a running master (master id already active)", MessageLevel::ERROR);
            } else {
                print_message("[SHM] Cannot lock " + pd_out + ": " + std::strerror(errno), MessageLevel::ERROR);
            }
            close(pd_output_fd_);
            pd_output_fd_ = -1;
            rollbackOwnedPdInput();
            return false;
        }
        owns_pd_output_ = true;
        if (ftruncate(pd_output_fd_, static_cast<off_t>(pdOutputSize)) != 0) {
            print_message("[SHM] Cannot resize " + pd_out + ": " + std::strerror(errno), MessageLevel::ERROR);
            rollbackOwnedPdOutput();
            rollbackOwnedPdInput();
            return false;
        }
        pdOutputPtr = mmap(nullptr, static_cast<std::size_t>(pdOutputSize), PROT_READ | PROT_WRITE, MAP_SHARED, pd_output_fd_, 0);
        if (pdOutputPtr == MAP_FAILED) {
            print_message("[SHM] Cannot map " + pd_out + ": " + std::strerror(errno), MessageLevel::ERROR);
            pdOutputPtr = nullptr;
            rollbackOwnedPdOutput();
            rollbackOwnedPdInput();
            return false;
        }
        pd_output_size_ = static_cast<std::size_t>(pdOutputSize);

        // Explicitly zero both buffers regardless of kernel page-allocation
        // guarantees: the master always starts from a known-clean state and
        // slaves receive zero output on the very first cycle.
        std::memset(pdInputPtr,  0, pd_input_size_);
        std::memset(pdOutputPtr, 0, pd_output_size_);

        print_message("[SHM] pd_input " + pd_in +
                      " size=" + std::to_string(pd_input_size_) + " bytes"
                      " pd_output " + pd_out +
                      " size=" + std::to_string(pd_output_size_) + " bytes",
                      MessageLevel::NORMAL);
        return true;
    }

    /// @brief Notifies all waiting clients that a new cycle is ready.
    bool notifyClients() noexcept {
        bool ok = true;
        for (auto &sem : sem_mutex) {
            if (sem == nullptr || sem == SEM_FAILED) {
                ok = false;
                continue;
            }
            int val = 0;
            if (sem_getvalue(sem, &val) != 0) {
                ok = false;
                continue;
            }
            if (val < 1 && sem_post(sem) != 0) {
                ok = false;
            }
        }
        return ok;
    }

    /// @brief Compatibility wrapper for notifyClients() (misspelled name preserved).
    void updateSempahore() {
        (void)notifyClients();
    }

    // -----------------------------------------------------------------------
    // Common: open existing shared memory (used by both sides)
    // -----------------------------------------------------------------------
    /**
     * @brief Opens and maps the existing `ecm{id}` bus shared memory and semaphores.
     * @return True when all objects are opened and mapped.
     */
    bool getSharedMemory() {
        closeSemaphores();
        releaseEcm();

        for (int i = 0; i < EC_SEM_NUM; i++) {
            const std::string semName = semaphoreName(i);
            sem_mutex[i] = sem_open(semName.c_str(), 0);
            if (sem_mutex[i] == SEM_FAILED) {
                print_message("[SHM] Cannot open semaphore " + semName, MessageLevel::ERROR);
                closeSemaphores();
                return false;
            }
        }

        const std::string shm_name = toPosixName(ecmName);
        ecm_fd_ = shm_open(shm_name.c_str(), O_RDWR, 0);
        if (ecm_fd_ < 0) {
            print_message("[SHM] Cannot open " + shm_name + ": " + std::strerror(errno), MessageLevel::ERROR);
            closeSemaphores();
            return false;
        }

        struct stat st{};
        if (fstat(ecm_fd_, &st) != 0) {
            print_message("[SHM] Cannot stat " + shm_name + ": " + std::strerror(errno), MessageLevel::ERROR);
            releaseEcm();
            closeSemaphores();
            return false;
        }

        if (!isValidMappedSize(st.st_size) || static_cast<std::size_t>(st.st_size) < sizeof(EcatBus)) {
            print_message("[SHM] Invalid size for " + shm_name, MessageLevel::ERROR);
            releaseEcm();
            closeSemaphores();
            return false;
        }
        ecm_size_ = static_cast<std::size_t>(st.st_size);

        void *addr = mmap(nullptr, ecm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, ecm_fd_, 0);
        if (addr == MAP_FAILED) {
            print_message("[SHM] Cannot map " + shm_name + ": " + std::strerror(errno), MessageLevel::ERROR);
            releaseEcm();
            closeSemaphores();
            return false;
        }
        ecatBus = static_cast<EcatBus *>(addr);

        return true;
    }

    /**
     * @brief Opens and maps the existing `pd_input{id}` and `pd_output{id}` buffers.
     * @return True on success.
     */
    bool getPdDataMemoryProvider() {
        releasePdInput();
        releasePdOutput();

        const std::string pd_in  = toPosixName(pdInputName);
        const std::string pd_out = toPosixName(pdOutputName);

        pd_input_fd_ = shm_open(pd_in.c_str(), O_RDWR, 0);
        if (pd_input_fd_ < 0) {
            print_message("[SHM] Cannot open " + pd_in + ": " + std::strerror(errno), MessageLevel::ERROR);
            return false;
        }
        struct stat st_in{};
        if (fstat(pd_input_fd_, &st_in) != 0 || !isValidMappedSize(st_in.st_size)) {
            print_message("[SHM] Invalid size for " + pd_in, MessageLevel::ERROR);
            releasePdInput();
            return false;
        }
        pd_input_size_ = static_cast<std::size_t>(st_in.st_size);

        pdInputPtr = mmap(nullptr, pd_input_size_, PROT_READ | PROT_WRITE, MAP_SHARED, pd_input_fd_, 0);
        if (pdInputPtr == MAP_FAILED) {
            print_message("[SHM] Cannot map " + pd_in + ": " + std::strerror(errno), MessageLevel::ERROR);
            pdInputPtr = nullptr;
            releasePdInput();
            return false;
        }

        pd_output_fd_ = shm_open(pd_out.c_str(), O_RDWR, 0);
        if (pd_output_fd_ < 0) {
            print_message("[SHM] Cannot open " + pd_out + ": " + std::strerror(errno), MessageLevel::ERROR);
            releasePdInput();
            return false;
        }
        struct stat st_out{};
        if (fstat(pd_output_fd_, &st_out) != 0 || !isValidMappedSize(st_out.st_size)) {
            print_message("[SHM] Invalid size for " + pd_out, MessageLevel::ERROR);
            releasePdOutput();
            releasePdInput();
            return false;
        }
        pd_output_size_ = static_cast<std::size_t>(st_out.st_size);

        pdOutputPtr = mmap(nullptr, pd_output_size_, PROT_READ | PROT_WRITE, MAP_SHARED, pd_output_fd_, 0);
        if (pdOutputPtr == MAP_FAILED) {
            print_message("[SHM] Cannot map " + pd_out + ": " + std::strerror(errno), MessageLevel::ERROR);
            pdOutputPtr = nullptr;
            releasePdOutput();
            releasePdInput();
            return false;
        }
        return true;
    }

    /**
     * @brief Connects the client to bus and PDO shared memory (exits on failure).
     */
    void init() {
        if (!getSharedMemory()) {
            print_message("[INIT] Cannot get shared memory.", MessageLevel::ERROR);
            exit(1);
        }
        if (!getPdDataMemoryProvider()) {
            print_message("[INIT] Cannot get PDO shared memory.", MessageLevel::ERROR);
            exit(1);
        }
        print_message("[SHM] Shared memory ready.", MessageLevel::NORMAL);
    }

    // -----------------------------------------------------------------------
    // Synchronisation
    // -----------------------------------------------------------------------
    /**
     * @brief Blocks on one semaphore slot until signaled.
     * @param id Semaphore slot in [0, EC_SEM_NUM).
     * @return False when the slot is invalid or sem_wait fails.
     */
    bool waitForSignal(int id = 0) noexcept {
        if (id < 0 || id >= EC_SEM_NUM || sem_mutex[id] == nullptr || sem_mutex[id] == SEM_FAILED) {
            return false;
        }
        while (sem_wait(sem_mutex[id]) != 0) {
            if (errno != EINTR) {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Assigns this thread a wait slot and blocks until notified.
     */
    void wait() {
        int slot = -1;
        {
            std::lock_guard<std::mutex> lk(thread_id_mutex_);
            const auto id = std::this_thread::get_id();
            const auto it = std::find(threadId_.begin(), threadId_.end(), id);
            if (it != threadId_.end()) {
                slot = static_cast<int>(std::distance(threadId_.begin(), it));
            } else {
                if (threadId_.size() >= EC_SEM_NUM) {
                    print_message("[SHM] Too many threads.", MessageLevel::ERROR);
                    return;
                }
                threadId_.push_back(id);
                slot = static_cast<int>(threadId_.size() - 1);
            }
        }

        if (!waitForSignal(slot)) {
            print_message("[SHM] Failed waiting on semaphore.", MessageLevel::ERROR);
        }
    }

    // -----------------------------------------------------------------------
    // EcatBus accessors (client helpers)
    // -----------------------------------------------------------------------
    double getBusMinCycleTime()     const { return ecatBus->min_cycle_time;     } ///< Minimum observed cycle [us].
    double getBusMaxCycleTime()     const { return ecatBus->max_cycle_time;     } ///< Maximum observed cycle [us].
    double getBusAvgCycleTime()     const { return ecatBus->avg_cycle_time;     } ///< Running average cycle [us].
    uint32_t getDt()                const { return ecatBus->dt;                 } ///< Configured cycle period [us].
    double getBusCurrentCycleTime() const { return ecatBus->current_cycle_time; } ///< Most recent cycle [us].
    bool   isAuthorized()           const { return ecatBus->is_authorized;      } ///< True when bus is healthy.
    long   getTimestamp()           const { return ecatBus->timestamp;          } ///< Last cycle monotonic timestamp [us].
    int    getSlaveNum()            const { return ecatBus->slave_num;          } ///< Number of slaves.
    int    getBusCurrentState()     const { return ecatBus->current_state;      } ///< Current bus state.

    void resetCycleTime()                { ecatBus->resetCycleTime = true;   } ///< Requests a cycle-time reset.
    void setBusRequestState(int state)   { ecatBus->request_state  = state;  } ///< Sets the client-requested state.

    /// @brief Returns the slave name by id.
    std::string getSlaveName(int slaveId) {
        return ecatBus->slaves[slaveId].name;
    }
    /// @brief Returns the full slave descriptor by id.
    Slave getSlave(int slaveId) {
        return ecatBus->slaves[slaveId];
    }
    /// @brief Returns the first slave matching a name, or an empty Slave.
    Slave findSlaveByName(const std::string &name) {
        for (int i = 0; i < ecatBus->slave_num; ++i)
            if (std::string(ecatBus->slaves[i].name) == name)
                return ecatBus->slaves[i];
        return {};
    }
    /// @brief Returns the id of the first slave matching a name, or -1.
    int findSlaveIdByName(const std::string &name) {
        for (int i = 0; i < ecatBus->slave_num; ++i)
            if (std::string(ecatBus->slaves[i].name) == name)
                return i;
        return -1;
    }
    /// @brief Returns an input variable name by slave and variable id.
    std::string getInputVarName(int slaveId, int varId)  const { return ecatBus->slaves[slaveId].input_vars[varId].name;  }
    /// @brief Returns an output variable name by slave and variable id.
    std::string getOutputVarName(int slaveId, int varId) const { return ecatBus->slaves[slaveId].output_vars[varId].name; }
    /// @brief Returns an input variable descriptor by slave and variable id.
    PdVar getSlaveInputVar(int slaveId, int varId)  { return ecatBus->slaves[slaveId].input_vars[varId];  }
    /// @brief Returns an output variable descriptor by slave and variable id.
    PdVar getSlaveOutputVar(int slaveId, int varId) { return ecatBus->slaves[slaveId].output_vars[varId]; }
    /// @brief Returns the first input variable matching a name, or an empty PdVar.
    PdVar findSlaveInputVarByName(int slaveId, const std::string &name) {
        for (int i = 0; i < ecatBus->slaves[slaveId].input_var_num; ++i)
            if (std::string(ecatBus->slaves[slaveId].input_vars[i].name) == name)
                return ecatBus->slaves[slaveId].input_vars[i];
        return {};
    }
    /// @brief Returns the id of the first input variable matching a name, or -1.
    int findSlaveInputVarIdByName(int slaveId, const std::string &name) {
        for (int i = 0; i < ecatBus->slaves[slaveId].input_var_num; ++i)
            if (std::string(ecatBus->slaves[slaveId].input_vars[i].name) == name)
                return i;
        return -1;
    }

    // -----------------------------------------------------------------------
    // Template PD access methods (by index)
    // -----------------------------------------------------------------------
    /// @brief Reads an input variable value by slave and variable id.
    /// @tparam T Value type; must match the variable's size in bytes.
    template<typename T> T getSlaveInputVarValue(int slaveId, int varId) {
        if (sizeof(T) != static_cast<std::size_t>(ecatBus->slaves[slaveId].input_vars[varId].size))
            print_message("Size mismatch", MessageLevel::WARNING);
        return *(T *)((char *)pdInputPtr + ecatBus->slaves[slaveId].input_vars[varId].offset);
    }
    /// @brief Writes an input variable value by slave and variable id.
    /// @tparam T Value type; must match the variable's size in bytes.
    template<typename T> void setSlaveInputVarValue(int slaveId, int varId, T value) {
        if (sizeof(T) != static_cast<std::size_t>(ecatBus->slaves[slaveId].input_vars[varId].size))
            print_message("Size mismatch", MessageLevel::WARNING);
        *(T *)((char *)pdInputPtr + ecatBus->slaves[slaveId].input_vars[varId].offset) = value;
    }
    /// @brief Reads an output variable value by slave and variable id.
    /// @tparam T Value type; must match the variable's size in bytes.
    template<typename T> T getSlaveOutputVarValue(int slaveId, int varId) {
        if (sizeof(T) != static_cast<std::size_t>(ecatBus->slaves[slaveId].output_vars[varId].size))
            print_message("Size mismatch", MessageLevel::WARNING);
        return *(T *)((char *)pdOutputPtr + ecatBus->slaves[slaveId].output_vars[varId].offset);
    }
    /// @brief Writes an output variable value by slave and variable id.
    /// @tparam T Value type; must match the variable's size in bytes.
    template<typename T> void setSlaveOutputVarValue(int slaveId, int varId, T value) {
        if (sizeof(T) != static_cast<std::size_t>(ecatBus->slaves[slaveId].output_vars[varId].size))
            print_message("Size mismatch", MessageLevel::WARNING);
        *(T *)((char *)pdOutputPtr + ecatBus->slaves[slaveId].output_vars[varId].offset) = value;
    }

    // Template PD access methods (by name)
    /// @brief Reads an input variable value by slave id and variable name.
    /// @tparam T Value type; must match the variable's size in bytes.
    template<typename T> T getSlaveInputVarValueByName(int slaveId, const std::string &name) {
        for (int i = 0; i < ecatBus->slaves[slaveId].input_var_num; ++i)
            if (strcmp(ecatBus->slaves[slaveId].input_vars[i].name, name.c_str()) == 0) {
                if (sizeof(T) != static_cast<std::size_t>(ecatBus->slaves[slaveId].input_vars[i].size))
                    print_message("Size mismatch", MessageLevel::WARNING);
                return *(T *)((char *)pdInputPtr + ecatBus->slaves[slaveId].input_vars[i].offset);
            }
        return std::numeric_limits<T>::max();
    }
    /// @brief Writes an input variable value by slave id and variable name.
    /// @tparam T Value type; must match the variable's size in bytes.
    template<typename T> void setSlaveInputVarValueByName(int slaveId, const std::string &name, T value) {
        for (int i = 0; i < ecatBus->slaves[slaveId].input_var_num; ++i)
            if (strcmp(ecatBus->slaves[slaveId].input_vars[i].name, name.c_str()) == 0) {
                if (sizeof(T) != static_cast<std::size_t>(ecatBus->slaves[slaveId].input_vars[i].size))
                    print_message("Size mismatch", MessageLevel::WARNING);
                *(T *)((char *)pdInputPtr + ecatBus->slaves[slaveId].input_vars[i].offset) = value;
            }
    }
    /// @brief Reads an output variable value by slave id and variable name.
    /// @tparam T Value type; must match the variable's size in bytes.
    template<typename T> T getSlaveOutputVarValueByName(int slaveId, const std::string &name) {
        for (int i = 0; i < ecatBus->slaves[slaveId].output_var_num; ++i)
            if (strcmp(ecatBus->slaves[slaveId].output_vars[i].name, name.c_str()) == 0) {
                if (sizeof(T) != static_cast<std::size_t>(ecatBus->slaves[slaveId].output_vars[i].size))
                    print_message("Size mismatch", MessageLevel::WARNING);
                return *(T *)((char *)pdOutputPtr + ecatBus->slaves[slaveId].output_vars[i].offset);
            }
        return std::numeric_limits<T>::max();
    }
    /// @brief Writes an output variable value by slave id and variable name.
    /// @tparam T Value type; must match the variable's size in bytes.
    template<typename T> void setSlaveOutputVarValueByName(int slaveId, const std::string &name, T value) {
        for (int i = 0; i < ecatBus->slaves[slaveId].output_var_num; ++i)
            if (strcmp(ecatBus->slaves[slaveId].output_vars[i].name, name.c_str()) == 0) {
                if (sizeof(T) != static_cast<std::size_t>(ecatBus->slaves[slaveId].output_vars[i].size))
                    print_message("Size mismatch", MessageLevel::WARNING);
                *(T *)((char *)pdOutputPtr + ecatBus->slaves[slaveId].output_vars[i].offset) = value;
            }
    }

    // Template PD pointer methods
    /// @brief Returns a typed pointer to an input variable by slave and variable id.
    /// @tparam T Value type; must match the variable's size in bytes.
    template<typename T> T *getSlaveInputVarPtr(int slaveId, int varId) {
        if (sizeof(T) != static_cast<std::size_t>(ecatBus->slaves[slaveId].input_vars[varId].size))
            print_message("Size mismatch", MessageLevel::WARNING);
        return (T *)((char *)pdInputPtr + ecatBus->slaves[slaveId].input_vars[varId].offset);
    }
    /// @brief Returns a typed pointer to an output variable by slave and variable id.
    /// @tparam T Value type; must match the variable's size in bytes.
    template<typename T> T *getSlaveOutputVarPtr(int slaveId, int varId) {
        if (sizeof(T) != static_cast<std::size_t>(ecatBus->slaves[slaveId].output_vars[varId].size))
            print_message("Size mismatch", MessageLevel::WARNING);
        return (T *)((char *)pdOutputPtr + ecatBus->slaves[slaveId].output_vars[varId].offset);
    }
    /// @brief Returns a typed pointer to an input variable by name, or nullptr.
    /// @tparam T Value type; must match the variable's size in bytes.
    template<typename T> T *findSlaveInputVarPtrByName(int slaveId, const std::string &name) {
        for (int i = 0; i < ecatBus->slaves[slaveId].input_var_num; ++i)
            if (strcmp(ecatBus->slaves[slaveId].input_vars[i].name, name.c_str()) == 0) {
                if (sizeof(T) != static_cast<std::size_t>(ecatBus->slaves[slaveId].input_vars[i].size))
                    print_message("Size mismatch", MessageLevel::WARNING);
                return (T *)((char *)pdInputPtr + ecatBus->slaves[slaveId].input_vars[i].offset);
            }
        return nullptr;
    }
    /// @brief Returns a typed pointer to an output variable by name, or nullptr.
    /// @tparam T Value type; must match the variable's size in bytes.
    template<typename T> T *findSlaveOutputVarPtrByName(int slaveId, const std::string &name) {
        for (int i = 0; i < ecatBus->slaves[slaveId].output_var_num; ++i)
            if (strcmp(ecatBus->slaves[slaveId].output_vars[i].name, name.c_str()) == 0) {
                if (sizeof(T) != static_cast<std::size_t>(ecatBus->slaves[slaveId].output_vars[i].size))
                    print_message("Size mismatch", MessageLevel::WARNING);
                return (T *)((char *)pdOutputPtr + ecatBus->slaves[slaveId].output_vars[i].offset);
            }
        return nullptr;
    }

    // -----------------------------------------------------------------------
    // Public data (accessible by both sides for direct struct manipulation)
    // -----------------------------------------------------------------------
    EcatBus *ecatBus    = nullptr; ///< Mapped bus snapshot (shared struct).
    void    *pdInputPtr  = nullptr; ///< Mapped `pd_input{id}` buffer.
    void    *pdOutputPtr = nullptr; ///< Mapped `pd_output{id}` buffer.
    sem_t   *sem_mutex[EC_SEM_NUM]{}; ///< Opened named semaphores.

    int         ecm_fd_{-1};                          ///< Bus shared-memory descriptor.
    int         pd_input_fd_{-1};                     ///< Input buffer descriptor.
    int         pd_output_fd_{-1};                    ///< Output buffer descriptor.
    std::size_t ecm_size_{EC_SHM_MAX_SIZE};           ///< Bus mapping size.
    std::size_t pd_input_size_{EC_SHM_MAX_SIZE};      ///< Input mapping size.
    std::size_t pd_output_size_{EC_SHM_MAX_SIZE};     ///< Output mapping size.

private:
    std::string ecmName{EC_SHM};
    std::string mutexName{EC_SEM_MUTEX};
    std::string pdInputName{"pd_input"};
    std::string pdOutputName{"pd_output"};
    bool owns_ecm_{false};
    bool owns_pd_input_{false};
    bool owns_pd_output_{false};
    bool owns_semaphores_{false};
    std::mutex thread_id_mutex_;

    std::vector<std::thread::id> threadId_;

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------
    std::string semaphoreName(int index) const {
        return toPosixName(mutexName + std::to_string(index));
    }

    static std::string toPosixName(const std::string &name) {
        return (name.front() == '/') ? name : "/" + name;
    }

    static bool isValidRegionSize(int size) {
        return size > 0 && size <= EC_SHM_MAX_SIZE;
    }

    static bool isValidMappedSize(off_t size) {
        return size > 0 && size <= static_cast<off_t>(EC_SHM_MAX_SIZE);
    }

    void releaseEcm() noexcept {
        if (ecatBus != nullptr) {
            if (munmap(ecatBus, ecm_size_) != 0) {
                print_message("[SHM] Cannot unmap " + ecmName + ": " + std::strerror(errno), MessageLevel::ERROR);
            }
            ecatBus = nullptr;
        }
        if (ecm_fd_ >= 0) {
            close(ecm_fd_);
            ecm_fd_ = -1;
        }
    }

    void releasePdInput() noexcept {
        if (pdInputPtr != nullptr) {
            if (munmap(pdInputPtr, pd_input_size_) != 0) {
                print_message("[SHM] Cannot unmap " + pdInputName + ": " + std::strerror(errno), MessageLevel::ERROR);
            }
            pdInputPtr = nullptr;
        }
        if (pd_input_fd_ >= 0) {
            close(pd_input_fd_);
            pd_input_fd_ = -1;
        }
    }

    void releasePdOutput() noexcept {
        if (pdOutputPtr != nullptr) {
            if (munmap(pdOutputPtr, pd_output_size_) != 0) {
                print_message("[SHM] Cannot unmap " + pdOutputName + ": " + std::strerror(errno), MessageLevel::ERROR);
            }
            pdOutputPtr = nullptr;
        }
        if (pd_output_fd_ >= 0) {
            close(pd_output_fd_);
            pd_output_fd_ = -1;
        }
    }

    void closeSemaphores() noexcept {
        for (auto &mutex : sem_mutex) {
            if (mutex != nullptr && mutex != SEM_FAILED) {
                if (sem_close(mutex) != 0) {
                    print_message("[SHM] Cannot close semaphore: " + std::string(std::strerror(errno)), MessageLevel::ERROR);
                }
            }
            mutex = nullptr;
        }
    }

    void rollbackCreatedSemaphores(int count) noexcept {
        for (int i = 0; i < count; ++i) {
            if (sem_mutex[i] != nullptr && sem_mutex[i] != SEM_FAILED) {
                if (sem_close(sem_mutex[i]) != 0) {
                    print_message("[SHM] Cannot close semaphore: " + std::string(std::strerror(errno)), MessageLevel::ERROR);
                }
            }
            sem_mutex[i] = nullptr;
            const std::string semName = semaphoreName(i);
            if (sem_unlink(semName.c_str()) != 0 && errno != ENOENT) {
                print_message("[SHM] Cannot unlink semaphore " + semName + ": " + std::strerror(errno), MessageLevel::ERROR);
            }
        }
    }

    void unlinkOwnedResources() noexcept {
        if (owns_semaphores_) {
            for (int i = 0; i < EC_SEM_NUM; ++i) {
                const std::string semName = semaphoreName(i);
                if (sem_unlink(semName.c_str()) != 0 && errno != ENOENT) {
                    print_message("[SHM] Cannot unlink semaphore " + semName + ": " + std::strerror(errno), MessageLevel::ERROR);
                }
            }
            owns_semaphores_ = false;
        }
        if (owns_ecm_) {
            unlinkSharedMemory(toPosixName(ecmName));
            owns_ecm_ = false;
        }
        if (owns_pd_input_) {
            unlinkSharedMemory(toPosixName(pdInputName));
            owns_pd_input_ = false;
        }
        if (owns_pd_output_) {
            unlinkSharedMemory(toPosixName(pdOutputName));
            owns_pd_output_ = false;
        }
    }

    void unlinkSharedMemory(const std::string &name) const noexcept {
        if (shm_unlink(name.c_str()) != 0 && errno != ENOENT) {
            print_message("[SHM] Cannot unlink " + name + ": " + std::strerror(errno), MessageLevel::ERROR);
        }
    }

    void rollbackOwnedEcm() noexcept {
        releaseEcm();
        if (owns_ecm_) {
            unlinkSharedMemory(toPosixName(ecmName));
            owns_ecm_ = false;
        }
    }

    void rollbackOwnedPdInput() noexcept {
        releasePdInput();
        if (owns_pd_input_) {
            unlinkSharedMemory(toPosixName(pdInputName));
            owns_pd_input_ = false;
        }
    }

    void rollbackOwnedPdOutput() noexcept {
        releasePdOutput();
        if (owns_pd_output_) {
            unlinkSharedMemory(toPosixName(pdOutputName));
            owns_pd_output_ = false;
        }
    }

    void rollbackOwnedSemaphores() noexcept {
        closeSemaphores();
        if (owns_semaphores_) {
            for (int i = 0; i < EC_SEM_NUM; ++i) {
                const std::string semName = semaphoreName(i);
                if (sem_unlink(semName.c_str()) != 0 && errno != ENOENT) {
                    print_message("[SHM] Cannot unlink semaphore " + semName + ": " + std::strerror(errno), MessageLevel::ERROR);
                }
            }
            owns_semaphores_ = false;
        }
    }

    enum class MessageLevel { NORMAL, WARNING, ERROR };

    void print_message(const std::string &msg, MessageLevel lvl) const {
        switch (lvl) {
            case MessageLevel::NORMAL:  std::cout << "\033[1;32m [INFO]";    break;
            case MessageLevel::WARNING: std::cout << "\033[1;33m [WARNING]"; break;
            case MessageLevel::ERROR:   std::cout << "\033[1;31m [ERROR]";   break;
        }
        std::cout << msg << "\033[0m" << std::endl;
    }
};

// --------------------------------------------------------------------------
// Backward-compatible type aliases
// --------------------------------------------------------------------------
using EcatConfigMaster = SharedMemoryConfig;
using EcatConfig       = SharedMemoryConfig;

}  // namespace rocos
