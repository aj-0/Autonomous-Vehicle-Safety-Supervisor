/*
 * access_control.c - role ownership table for server-side access control.
 */
#include <errno.h>
#include <signal.h>
#include <string.h>

#include "access_control.h"
#include "messages.h"

void access_init(access_roles_t *t)
{
    memset(t, 0, sizeof *t);
}

static access_client_t *find(access_roles_t *t, int32_t scoid)
{
    for (int i = 0; i < IPC_MAX_CLIENTS; i++)
        if (t->cl[i].used && t->cl[i].scoid == scoid)
            return &t->cl[i];
    return NULL;
}

static int pid_alive(int32_t pid)
{
    return pid > 0 && (kill(pid, 0) == 0 || errno == EPERM);
}

int access_hello(access_roles_t *t, int32_t scoid, int32_t pid, uint32_t roles, const char *name)
{
    uint32_t excl = roles & AV_ROLES_EXCLUSIVE;

    /* An exclusive role owned by another live process cannot be taken over. */
    for (int i = 0; i < IPC_MAX_CLIENTS; i++) {
        access_client_t *c = &t->cl[i];
        if (!c->used || c->scoid == scoid || !(c->roles & excl))
            continue;
        if (pid_alive(c->pid) && c->pid != pid)
            return EBUSY;
        c->roles &= ~excl;            /* stale owner (process already gone) */
    }

    access_client_t *c = find(t, scoid);
    if (c == NULL) {
        for (int i = 0; i < IPC_MAX_CLIENTS && c == NULL; i++)
            if (!t->cl[i].used)
                c = &t->cl[i];
        if (c == NULL)
            return ENOSPC;
        memset(c, 0, sizeof *c);
        c->used = 1;
        c->scoid = scoid;
    }
    c->pid = pid;
    c->roles |= roles;
    strncpy(c->name, name, sizeof c->name - 1);
    return 0;
}

int access_has_role(const access_roles_t *t, int32_t scoid, uint32_t role)
{
    for (int i = 0; i < IPC_MAX_CLIENTS; i++)
        if (t->cl[i].used && t->cl[i].scoid == scoid)
            return (t->cl[i].roles & role) != 0;
    return 0;
}

uint32_t access_drop(access_roles_t *t, int32_t scoid, int32_t *pid_out, char *name_out, int name_len)
{
    access_client_t *c = find(t, scoid);
    if (c == NULL)
        return 0;
    uint32_t roles = c->roles;
    if (pid_out)
        *pid_out = c->pid;
    if (name_out && name_len > 0) {
        strncpy(name_out, c->name, (size_t)name_len - 1);
        name_out[name_len - 1] = '\0';
    }
    memset(c, 0, sizeof *c);
    return roles;
}

int32_t access_role_pid(const access_roles_t *t, uint32_t role)
{
    for (int i = 0; i < IPC_MAX_CLIENTS; i++)
        if (t->cl[i].used && (t->cl[i].roles & role))
            return t->cl[i].pid;
    return 0;
}
