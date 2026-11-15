/*
 * websim: a small micro T-Kernel look-alike, just the calls app_program/ uses.
 *
 * Tasks are coroutines.  The JavaScript side (sim.js) runs the scheduler
 * loop: it asks kernel_pick() which task is ready, runs it with Asyncify
 * until it blocks (every blocking call ends in sim_yield()), then asks again.
 * Priorities follow the kernel: 1 is the highest, FIFO within a priority,
 * and a task readied by a higher-priority wake-up preempts at once
 * (see preempt_check()).
 *
 * Time is the kernel tick (CNF_TIMER_PERIOD = 10 ms).  The JS side calls
 * kernel_tick() once every 10 ms of simulated time.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#define MAX_TSK    16
#define MAX_FLG    8
#define MAX_CYC    8
#define MAX_INT    32
#define ASTACK_SZ  (24 * 1024) /* Asyncify save area per task */
#define TSTACK_SZ  (20 * 1024) /* C shadow stack per task */

typedef struct
{
    int    used;
    FP     entry;
    PRI    pri;
    int    state;    /* TTS_DMT, TTS_RDY, TTS_RUN, TTS_WAI */
    int    wait;     /* TTW_SLP, TTW_DLY, TTW_FLG or 0 */
    int    wup;      /* queued wake-ups */
    uint32_t seq;    /* order of becoming ready, for FIFO */
    int64_t  wake_at; /* tick when a timed wait ends, -1 = never */
    ER       result;
    int      suspended; /* saved by Asyncify, resume with a rewind */
    uint32_t sp;     /* saved C shadow stack pointer */
    /* event flag wait */
    ID       flg_id;
    UINT     flg_ptn;
    UINT     flg_mode;
    UINT     flg_got;
} tsk_t;

typedef struct
{
    int  used;
    UINT ptn;
} flg_t;

typedef struct
{
    int  used;
    int  active;
    FP   handler;
    void *exinf;
    int64_t period;
    int64_t next;
    int64_t phase;
} cyc_t;

static tsk_t g_tsk[MAX_TSK + 1];
static flg_t g_flg[MAX_FLG + 1];
static cyc_t g_cyc[MAX_CYC + 1];
static FP    g_int[MAX_INT];
static int   g_cur;             /* running task, 0 = none (interrupt / tick) */
static int   g_in_handler;      /* inside an interrupt or cyclic handler */
static int64_t  g_tick;
static uint32_t g_seq;

static uint8_t g_astack[MAX_TSK + 1][ASTACK_SZ] __attribute__((aligned(16)));
static uint8_t g_tstack[MAX_TSK + 1][TSTACK_SZ] __attribute__((aligned(16)));

#define TICK_MS CNF_TIMER_PERIOD

/* ---- helpers --------------------------------------------------------- */
static void make_ready(int idx)
{
    g_tsk[idx].state = TTS_RDY;
    g_tsk[idx].wait  = 0;
    g_tsk[idx].seq   = ++g_seq;
}

/* Stop running the current task until it is picked again */
static void block_current(void)
{
    tsk_t *t = &g_tsk[g_cur];

    t->suspended = 1;
    sim_yield(); /* unwinds to JS; returns here when resumed */
    t->suspended = 0;
}

/* A task of higher priority than the running one just became ready */
static void preempt_check(int woken)
{
    if ((g_cur != 0) && (0 == g_in_handler) &&
        (g_tsk[woken].pri < g_tsk[g_cur].pri))
    {
        g_tsk[g_cur].state = TTS_RDY;
        g_tsk[g_cur].seq   = ++g_seq;
        block_current();
    }
}

static int64_t ms_to_ticks(RELTIM ms)
{
    /* the kernel rounds up to the tick, and a wait ends on the tick AFTER
       the one it was started in: (ms + tick - 1) / tick + 1 */
    return ((int64_t)ms + TICK_MS - 1) / TICK_MS + 1;
}

/* ---- system ---------------------------------------------------------- */
ER tk_ref_ver(T_RVER *p)
{
    p->maker = 0;
    p->prid  = 0;
    p->spver = 0x0300;
    p->prver = 0x0003;
    return E_OK;
}

INT tk_get_prc(void)
{
    return 1;
}

/* ---- tasks ----------------------------------------------------------- */
ID tk_cre_tsk(const T_CTSK *p)
{
    int i;

    for (i = 1; i <= MAX_TSK; i++)
    {
        if (!g_tsk[i].used)
        {
            g_tsk[i].used  = 1;
            g_tsk[i].entry = p->task;
            g_tsk[i].pri   = p->itskpri;
            g_tsk[i].state = TTS_DMT;
            return i;
        }
    }
    return E_LIMIT;
}

ER tk_sta_tsk(ID id, INT stacd)
{
    (void)stacd;
    if ((id < 1) || (id > MAX_TSK) || !g_tsk[id].used)
    {
        return E_ID;
    }
    g_tsk[id].wup       = 0;
    g_tsk[id].suspended = 0;
    g_tsk[id].sp        = (uint32_t)(g_tstack[id] + TSTACK_SZ);
    make_ready(id);
    preempt_check(id);
    return E_OK;
}

ER tk_slp_tsk(TMO tmout)
{
    tsk_t *t = &g_tsk[g_cur];

    if (t->wup > 0)
    {
        t->wup--;
        return E_OK;
    }
    if (0 == tmout)
    {
        return E_TMOUT;
    }
    t->state   = TTS_WAI;
    t->wait    = TTW_SLP;
    t->wake_at = (tmout < 0) ? -1 : (g_tick + ms_to_ticks((RELTIM)tmout));
    t->result  = E_OK;
    block_current();
    return t->result;
}

ER tk_dly_tsk(RELTIM dlytim)
{
    tsk_t *t = &g_tsk[g_cur];

    t->state   = TTS_WAI;
    t->wait    = TTW_DLY;
    t->wake_at = g_tick + ms_to_ticks(dlytim);
    t->result  = E_OK;
    block_current();
    return t->result;
}

ER tk_wup_tsk(ID id)
{
    tsk_t *t;

    if (TSK_SELF == id)
    {
        id = g_cur;
    }
    if ((id < 1) || (id > MAX_TSK) || !g_tsk[id].used)
    {
        return E_ID;
    }
    t = &g_tsk[id];
    if ((TTS_WAI == t->state) && (TTW_SLP == t->wait))
    {
        t->result = E_OK;
        make_ready(id);
        preempt_check(id);
    }
    else
    {
        t->wup++;
    }
    return E_OK;
}

INT tk_can_wup(ID id)
{
    INT n;

    if (TSK_SELF == id)
    {
        id = g_cur;
    }
    n             = g_tsk[id].wup;
    g_tsk[id].wup = 0;
    return n;
}

/* ---- event flags ----------------------------------------------------- */
ID tk_cre_flg(const T_CFLG *p)
{
    int i;

    for (i = 1; i <= MAX_FLG; i++)
    {
        if (!g_flg[i].used)
        {
            g_flg[i].used = 1;
            g_flg[i].ptn  = p->iflgptn;
            return i;
        }
    }
    return E_LIMIT;
}

static int flg_ok(const tsk_t *t, UINT ptn)
{
    if (t->flg_mode & TWF_ORW)
    {
        return (ptn & t->flg_ptn) != 0u;
    }
    return (ptn & t->flg_ptn) == t->flg_ptn;
}

ER tk_set_flg(ID id, UINT setptn)
{
    int i;
    int woke = 0;

    if ((id < 1) || (id > MAX_FLG) || !g_flg[id].used)
    {
        return E_ID;
    }
    g_flg[id].ptn |= setptn;

    for (i = 1; i <= MAX_TSK; i++)
    {
        tsk_t *t = &g_tsk[i];

        if (t->used && (TTS_WAI == t->state) && (TTW_FLG == t->wait) &&
            (t->flg_id == id) && flg_ok(t, g_flg[id].ptn))
        {
            t->flg_got = g_flg[id].ptn;
            if (t->flg_mode & TWF_CLR)
            {
                g_flg[id].ptn = 0;
            }
            else if (t->flg_mode & TWF_BITCLR)
            {
                g_flg[id].ptn &= ~t->flg_ptn;
            }
            t->result = E_OK;
            make_ready(i);
            woke = i;
        }
    }
    if (woke != 0)
    {
        preempt_check(woke);
    }
    return E_OK;
}

ER tk_clr_flg(ID id, UINT clrptn)
{
    if ((id < 1) || (id > MAX_FLG) || !g_flg[id].used)
    {
        return E_ID;
    }
    g_flg[id].ptn &= clrptn;
    return E_OK;
}

ER tk_wai_flg(ID id, UINT waiptn, UINT wfmode, UINT *p_flgptn, TMO tmout)
{
    tsk_t *t = &g_tsk[g_cur];

    if ((id < 1) || (id > MAX_FLG) || !g_flg[id].used)
    {
        return E_ID;
    }
    t->flg_id   = id;
    t->flg_ptn  = waiptn;
    t->flg_mode = wfmode;

    if (flg_ok(t, g_flg[id].ptn))
    {
        *p_flgptn = g_flg[id].ptn;
        if (wfmode & TWF_CLR)
        {
            g_flg[id].ptn = 0;
        }
        else if (wfmode & TWF_BITCLR)
        {
            g_flg[id].ptn &= ~waiptn;
        }
        return E_OK;
    }
    if (0 == tmout)
    {
        return E_TMOUT;
    }
    t->state   = TTS_WAI;
    t->wait    = TTW_FLG;
    t->wake_at = (tmout < 0) ? -1 : (g_tick + ms_to_ticks((RELTIM)tmout));
    t->result  = E_OK;
    block_current();
    if (E_OK == t->result)
    {
        *p_flgptn = t->flg_got;
    }
    return t->result;
}

/* ---- cyclic handlers and interrupts ---------------------------------- */
ID tk_cre_cyc(const T_CCYC *p)
{
    int i;

    for (i = 1; i <= MAX_CYC; i++)
    {
        if (!g_cyc[i].used)
        {
            g_cyc[i].used    = 1;
            g_cyc[i].active  = 0;
            g_cyc[i].handler = p->cychdr;
            g_cyc[i].exinf   = p->exinf;
            g_cyc[i].period  = ms_to_ticks(p->cyctim) - 1;
            g_cyc[i].phase   = ms_to_ticks(p->cycphs) - 1;
            return i;
        }
    }
    return E_LIMIT;
}

ER tk_sta_cyc(ID id)
{
    if ((id < 1) || (id > MAX_CYC) || !g_cyc[id].used)
    {
        return E_ID;
    }
    g_cyc[id].active = 1;
    g_cyc[id].next   = g_tick + g_cyc[id].phase;
    return E_OK;
}

ER tk_def_int(UINT intno, const T_DINT *p)
{
    if (intno >= MAX_INT)
    {
        return E_PAR;
    }
    g_int[intno] = p->inthdr;
    return E_OK;
}

/* ---- called by sim.js ------------------------------------------------ */
extern int32_t usermain(void);

/* tasks are called as (stacd, exinf); usermain takes nothing, and a wasm
   indirect call needs the exact signature */
static void boot_task(INT stacd, void *exinf)
{
    (void)stacd; (void)exinf;
    usermain();
}

__attribute__((export_name("kernel_boot")))
void kernel_boot(void)
{
    T_CTSK ct = {0};

    ct.task    = (FP)boot_task;
    ct.itskpri = 1;
    tk_sta_tsk(tk_cre_tsk(&ct), 0);
}

/* One kernel tick (10 ms): wake timed waits, fire cyclic handlers. */
__attribute__((export_name("kernel_tick")))
void kernel_tick(void)
{
    int i;

    g_tick++;
    g_in_handler++;

    for (i = 1; i <= MAX_TSK; i++)
    {
        tsk_t *t = &g_tsk[i];

        if (t->used && (TTS_WAI == t->state) && (t->wake_at >= 0) &&
            (t->wake_at <= g_tick))
        {
            /* a plain delay completes; any other timed wait times out */
            t->result = (TTW_DLY == t->wait) ? E_OK : E_TMOUT;
            make_ready(i);
        }
    }
    for (i = 1; i <= MAX_CYC; i++)
    {
        cyc_t *c = &g_cyc[i];

        if (c->used && c->active && (c->next <= g_tick))
        {
            c->next += c->period;
            ((void (*)(void *))c->handler)(c->exinf);
        }
    }
    g_in_handler--;
}

__attribute__((export_name("kernel_irq")))
void kernel_irq(int intno)
{
    if ((intno >= 0) && (intno < MAX_INT) && g_int[intno])
    {
        g_in_handler++;
        ((void (*)(UINT))g_int[intno])((UINT)intno);
        g_in_handler--;
    }
}

/* Next task to run, or 0.  Marks it running. */
__attribute__((export_name("kernel_pick")))
int kernel_pick(void)
{
    int best = 0;
    int i;

    for (i = 1; i <= MAX_TSK; i++)
    {
        tsk_t *t = &g_tsk[i];

        if (t->used && (TTS_RDY == t->state))
        {
            if ((0 == best) || (t->pri < g_tsk[best].pri) ||
                ((t->pri == g_tsk[best].pri) && (t->seq < g_tsk[best].seq)))
            {
                best = i;
            }
        }
    }
    if (best != 0)
    {
        g_tsk[best].state = TTS_RUN;
        g_cur             = best;
    }
    return best;
}

__attribute__((export_name("task_tramp")))
void task_tramp(int idx)
{
    g_tsk[idx].entry(0, 0);
    /* the task function returned: it has ended */
    g_tsk[idx].state = TTS_DMT;
}

__attribute__((export_name("kernel_after")))
void kernel_after(void)
{
    g_cur = 0;
}

__attribute__((export_name("task_suspended")))
int task_suspended(int idx) { return g_tsk[idx].suspended; }
__attribute__((export_name("task_astack")))
uint32_t task_astack(int idx) { return (uint32_t)g_astack[idx]; }
__attribute__((export_name("task_astack_size")))
uint32_t task_astack_size(void) { return ASTACK_SZ; }
__attribute__((export_name("task_get_sp")))
uint32_t task_get_sp(int idx) { return g_tsk[idx].sp; }
__attribute__((export_name("task_set_sp")))
void task_set_sp(int idx, uint32_t sp) { g_tsk[idx].sp = sp; }
__attribute__((export_name("kernel_cur")))
int kernel_cur(void) { return g_cur; }
__attribute__((export_name("kernel_ticks")))
uint32_t kernel_ticks(void) { return (uint32_t)g_tick; }
