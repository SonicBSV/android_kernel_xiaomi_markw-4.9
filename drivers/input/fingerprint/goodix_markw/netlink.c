#include <linux/init.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/types.h>
#include <net/sock.h>
#include <net/netlink.h>

#define NETLINK_TEST    25
#define MAX_MSGSIZE     (4 * 1024)

static int pid;
static struct sock *nl_sk;
static DEFINE_MUTEX(nl_lock);

struct gf_uk_channel {
    int channel_id;
    int reserved;
    char buf[3 * 1024];
    int len;
};

void sendnlmsg(char *message)
{
    struct sk_buff *skb;
    struct nlmsghdr *nlh;
    int len = NLMSG_SPACE(MAX_MSGSIZE);
    int slen;

    if (!message || !nl_sk || !pid)
        return;

    mutex_lock(&nl_lock);

    skb = alloc_skb(len, GFP_ATOMIC);
    if (!skb) {
        pr_err("gf_netlink: alloc_skb error\n");
        mutex_unlock(&nl_lock);
        return;
    }

    slen = strlen(message);
    if (slen >= MAX_MSGSIZE)
        slen = MAX_MSGSIZE - 1;

    nlh = nlmsg_put(skb, 0, 0, 0, MAX_MSGSIZE, 0);
    if (!nlh) {
        kfree_skb(skb);
        mutex_unlock(&nl_lock);
        return;
    }

    NETLINK_CB(skb).portid = 0;
    NETLINK_CB(skb).dst_group = 0;

    memcpy(NLMSG_DATA(nlh), message, slen);
    ((char *)NLMSG_DATA(nlh))[slen] = '\0';

    netlink_unicast(nl_sk, skb, pid, MSG_DONTWAIT);

    mutex_unlock(&nl_lock);
}

static void nl_data_ready(struct sk_buff *__skb)
{
    struct sk_buff *skb;
    struct nlmsghdr *nlh;
    char str[100];

    skb = skb_get(__skb);
    if (!skb || skb->len < NLMSG_SPACE(0)) {
        if (skb)
            kfree_skb(skb);
        return;
    }

    nlh = nlmsg_hdr(skb);
    memcpy(str, NLMSG_DATA(nlh), min_t(size_t, sizeof(str) - 1, skb->len));
    str[sizeof(str) - 1] = '\0';
    
    mutex_lock(&nl_lock);
    pid = nlh->nlmsg_pid;
    mutex_unlock(&nl_lock);

    kfree_skb(skb);
}

int netlink_init(void)
{
    struct netlink_kernel_cfg netlink_cfg = {
        .groups = 0,
        .flags = 0,
        .input = nl_data_ready,
        .cb_mutex = NULL,
    };

    nl_sk = netlink_kernel_create(&init_net, NETLINK_TEST, &netlink_cfg);
    if (!nl_sk) {
        pr_err("gf_netlink: failed to create netlink socket\n");
        return -ENOMEM;
    }

    pr_info("gf_netlink: initialized\n");
    return 0;
}

void netlink_exit(void)
{
    if (nl_sk) {
        netlink_kernel_release(nl_sk);
        nl_sk = NULL;
    }
    pr_info("gf_netlink: exited\n");
}
