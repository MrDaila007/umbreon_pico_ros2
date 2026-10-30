/* Fake rcl/rclc lifecycle backend for host tests: records every create/destroy,
 * injects a failure at any step, models an owned transport socket and a dead
 * agent. Holds no real memory; "live" is a bitmap of created entities. */
#pragma once
#include "ros_entities.h"
#define FAKE_ROS_LOG 128
typedef struct {
    int fail_create_id;          /* -1: none; else this id's create fails */
    bool agent_alive, release_ok, sync_ok;
    ros_spin_result_t spin_result;
    int64_t epoch_ns;            /* sync result at mono_ms */
    uint64_t mono_ms;
    /* observations */
    char op[FAKE_ROS_LOG];       /* 'c' create, 'd' destroy, 't' teardown begin, 'r' release */
    int id[FAKE_ROS_LOG];
    unsigned len;
    uint32_t live;
    unsigned destroy_without_create, creates_while_live_transport;
    unsigned teardown_after_destroy, destroys_before_teardown;
    bool transport_open, teardown_begun;
    unsigned spins, pings, syncs, session_starts, creates;
    uint32_t generation;
    ros_session_config_t last_cfg;
    void (*on_destroy)(void *user, ros_entity_id_t id);
    void (*on_spin)(void *user);
    void (*on_blocking)(void *user); /* create, ping and sync: bounded blocking calls */
    void *user;
} fake_ros_t;
void fake_ros_reset(fake_ros_t *f);
ros_backend_t fake_ros_backend(fake_ros_t *f);
unsigned fake_ros_count(const fake_ros_t *f, char op);
