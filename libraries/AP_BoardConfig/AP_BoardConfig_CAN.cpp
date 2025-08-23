Inside init() function
AP_Proximity *prx = AP::proximity();
RangeFinder *rnd = AP::rangefinder();
if(prx->get_type(0) == AP_Proximity::Type::IRadar || rnd->get_rnd() ==RangeFinder::RangeFinder_Type::RangeFinder_TYPE_IOTECH_Radar)
        {
            IR24_Backend *_ir24_backend = AP::__backend();
            _ir24_backend->ir24_init_thread();
        }
