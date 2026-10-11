/*
 * access_control.h - who is allowed to send what.
 *
 * Every client connection (identified by its server connection id "scoid") claims roles
 * with AV_MSG_HELLO. Exclusive roles (NAV, RECOVERY, SENSOR, PERCEPTION, SUPERVISOR) can
 * be owned by only one live process. Servers reject messages from connections that do not
 * own the matching role, so the command gate is enforced by the server, not by convention.
 */
#ifndef ACCESS_CONTROL_H
#define ACCESS_CONTROL_H

#include <stdint.h>

#define IPC_MAX_CLIENTS 16

typedef struct {
    int      used;
    int32_t  scoid;
    int32_t  pid;
    uint32_t roles;
    char     name[16];
} access_client_t;

typedef struct {
    access_client_t cl[IPC_MAX_CLIENTS];
} access_roles_t;

void     access_init(access_roles_t *t);

/* Returns 0 on success or EBUSY if an exclusive role is owned by another live process. */
int      access_hello(access_roles_t *t, int32_t scoid, int32_t pid, uint32_t roles, const char *name);

int      access_has_role(const access_roles_t *t, int32_t scoid, uint32_t role);

/* Forget a connection (disconnect pulse). Returns the roles it held; pid_out may be NULL. */
uint32_t access_drop(access_roles_t *t, int32_t scoid, int32_t *pid_out, char *name_out, int name_len);

int32_t  access_role_pid(const access_roles_t *t, uint32_t role);

#endif /* ACCESS_CONTROL_H */
