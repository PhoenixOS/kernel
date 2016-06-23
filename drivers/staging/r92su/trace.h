#if !defined(_R92SU_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)

#include <net/cfg80211.h>
#include <linux/skbuff.h>
#include <linux/tracepoint.h>
#include "def.h"

#if !defined(_R92SU_TRACE_H)

#endif /* __R92SU_TRACE_H */

#define _R92SU_TRACE_H

/* create empty functions when tracing is disabled */
#if !defined(CONFIG_R92SU_TRACING)
#undef TRACE_EVENT
#define TRACE_EVENT(name, proto, ...) \
static inline void trace_ ## name(proto) {}
#undef DECLARE_EVENT_CLASS
#define DECLARE_EVENT_CLASS(...)
#undef DEFINE_EVENT
#define DEFINE_EVENT(evt_class, name, proto, ...) \
static inline void trace_ ## name(proto) {}
#endif /* !CONFIG_R92SU_TRACING || __CHECKER__ */

#define DEV_ENTRY	__string(dev, dev_name(dev))
#define DEV_ASSIGN	__assign_str(dev, dev_name(dev))

#undef TRACE_SYSTEM
#define TRACE_SYSTEM r92su_h2cc2h

TRACE_EVENT(r92su_h2c,
	TP_PROTO(const struct device *dev, const struct h2cc2h *h2c),

	TP_ARGS(dev, h2c),

	TP_STRUCT__entry(
		DEV_ENTRY
		__field(unsigned int, event)
		__field(unsigned int, cmd_seq)
		__field(size_t, len)
		__dynamic_array(u8, h2c, le16_to_cpu(h2c->len))
	),

	TP_fast_assign(
		DEV_ASSIGN;
		__entry->event = h2c->event;
		__entry->cmd_seq = h2c->cmd_seq;
		__entry->len = le16_to_cpu(h2c->len);
		memcpy(__get_dynamic_array(h2c), h2c, __entry->len);
	),

	TP_printk(
		"[%s] send cmd 0x%x, seq:%d, len %zd",
		__get_str(dev), __entry->event, __entry->cmd_seq, __entry->len
	)
);

TRACE_EVENT(r92su_c2h,
	TP_PROTO(const struct device *dev, const struct h2cc2h *c2h),

	TP_ARGS(dev, c2h),

	TP_STRUCT__entry(
		DEV_ENTRY
		__field(unsigned int, event)
		__field(unsigned int, cmd_seq)
		__field(size_t, len)
		__dynamic_array(u8, c2h, le16_to_cpu(c2h->len))
	),

	TP_fast_assign(
		DEV_ASSIGN;
		__entry->event = c2h->event;
		__entry->cmd_seq = c2h->cmd_seq;
		__entry->len = le16_to_cpu(c2h->len);
		memcpy(__get_dynamic_array(c2h), c2h, __entry->len);
	),

	TP_printk(
		"[%s] received event 0x%x, seq:%d, len %zd",
		__get_str(dev), __entry->event, __entry->cmd_seq, __entry->len
	)
);

#undef TRACE_SYSTEM
#define TRACE_SYSTEM r92su_io

TRACE_EVENT(r92su_ioread32,
	TP_PROTO(const struct device *dev, const u32 address, const u32 val),
	TP_ARGS(dev, address, val),
	TP_STRUCT__entry(
		DEV_ENTRY
		__field(u32, address)
		__field(u32, val)
	),
	TP_fast_assign(
		DEV_ASSIGN;
		__entry->address = address;
		__entry->val = val;
	),
	TP_printk("[%s] read32 io[%#x] = %#x",
		  __get_str(dev), __entry->address, __entry->val)
);

TRACE_EVENT(r92su_ioread16,
	TP_PROTO(const struct device *dev, const u32 address, const u16 val),
	TP_ARGS(dev, address, val),
	TP_STRUCT__entry(
		DEV_ENTRY
		__field(u32, address)
		__field(u16, val)
	),
	TP_fast_assign(
		DEV_ASSIGN;
		__entry->address = address;
		__entry->val = val;
	),
	TP_printk("[%s] read16 io[%#x] = %#x",
		  __get_str(dev), __entry->address, __entry->val)
);

TRACE_EVENT(r92su_ioread8,
	TP_PROTO(const struct device *dev, const u32 address, const u8 val),
	TP_ARGS(dev, address, val),
	TP_STRUCT__entry(
		DEV_ENTRY
		__field(u32, address)
		__field(u8, val)
	),
	TP_fast_assign(
		DEV_ASSIGN;
		__entry->address = address;
		__entry->val = val;
	),
	TP_printk("[%s] read8 io[%#x] = %#x",
		  __get_str(dev), __entry->address, __entry->val)
);

TRACE_EVENT(r92su_iowrite8,
	TP_PROTO(const struct device *dev, const u32 address, const u8 val),
	TP_ARGS(dev, address, val),
	TP_STRUCT__entry(
		DEV_ENTRY
		__field(u32, address)
		__field(u8, val)
	),
	TP_fast_assign(
		DEV_ASSIGN;
		__entry->address = address;
		__entry->val = val;
	),
	TP_printk("[%s] write8 io[%#x] = %#x)",
		  __get_str(dev), __entry->address, __entry->val)
);

TRACE_EVENT(r92su_iowrite16,
	TP_PROTO(const struct device *dev, const u32 address, const u16 val),
	TP_ARGS(dev, address, val),
	TP_STRUCT__entry(
		DEV_ENTRY
		__field(u32, address)
		__field(u16, val)
	),
	TP_fast_assign(
		DEV_ASSIGN;
		__entry->address = address;
		__entry->val = val;
	),
	TP_printk("[%s] write16 io[%#x] = %#x)",
		  __get_str(dev), __entry->address, __entry->val)
);

TRACE_EVENT(r92su_iowrite32,
	TP_PROTO(const struct device *dev, const u32 address, const u32 val),
	TP_ARGS(dev, address, val),
	TP_STRUCT__entry(
		DEV_ENTRY
		__field(u32, address)
		__field(u32, val)
	),
	TP_fast_assign(
		DEV_ASSIGN;
		__entry->address = address;
		__entry->val = val;
	),
	TP_printk("[%s] write32 io[%#x] = %#x)",
		  __get_str(dev), __entry->address, __entry->val)
);

#undef TRACE_SYSTEM
#define TRACE_SYSTEM r92su_data

TRACE_EVENT(r92su_tx_data,
	TP_PROTO(const struct device *dev,
		 const struct sk_buff *skb),
	TP_ARGS(dev, skb),
	TP_STRUCT__entry(
		DEV_ENTRY

		__dynamic_array(u8, data, skb->len)
	),
	TP_fast_assign(
		DEV_ASSIGN;
		memcpy(__get_dynamic_array(data), skb->data, skb->len);
	),
	TP_printk("[%s] TX frame data", __get_str(dev))
);

TRACE_EVENT(r92su_rx_data,
	TP_PROTO(const struct device *dev,
		 const struct sk_buff *skb),
	TP_ARGS(dev, skb),
	TP_STRUCT__entry(
		DEV_ENTRY

		__dynamic_array(u8, data, skb->len)
	),
	TP_fast_assign(
		DEV_ASSIGN;
		memcpy(__get_dynamic_array(data), skb->data, skb->len);
	),
	TP_printk("[%s] RX frame data", __get_str(dev))
);

#undef TRACE_SYSTEM
#define TRACE_SYSTEM r92su_msg

#define MAX_MSG_LEN     110

DECLARE_EVENT_CLASS(r92su_msg_event,
	TP_PROTO(const struct device *dev, struct va_format *vaf),
	TP_ARGS(dev, vaf),
	TP_STRUCT__entry(
		DEV_ENTRY

		__dynamic_array(char, msg, MAX_MSG_LEN)
	),
	TP_fast_assign(
		DEV_ASSIGN;
		WARN_ON_ONCE(vsnprintf(__get_dynamic_array(msg),
				       MAX_MSG_LEN, vaf->fmt,
				       *vaf->va) >= MAX_MSG_LEN);
	),
	TP_printk("[%s] %s", __get_str(dev), __get_str(msg))
);

DEFINE_EVENT(r92su_msg_event, r92su_err,
	TP_PROTO(const struct device *dev, struct va_format *vaf),
	TP_ARGS(dev, vaf)
);

DEFINE_EVENT(r92su_msg_event, r92su_info,
	TP_PROTO(const struct device *dev, struct va_format *vaf),
	TP_ARGS(dev, vaf)
);

DEFINE_EVENT(r92su_msg_event, r92su_dbg,
	TP_PROTO(const struct device *dev, struct va_format *vaf),
	TP_ARGS(dev, vaf)
);

#endif /* _R92SU_TRACE_H || TRACE_HEADER_MULTI_READ */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace

#include <trace/define_trace.h>
