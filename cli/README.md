for other unrecognized lines in dump, write warnning
the dump and upload sub commands need to be executed concurrently
implement "servo" get and set
dumping with cli is much faster less then 2 sec, with msp dump it 6 sec
get aux and save to files - fc_msp box_id doesn't match the cli box_id - opened PR https://github.com/iNavFlight/inav/pull/5654
refactor use ? to passthrough function result
get osd_layout and save to files - need to implement inav version of msp MSP2_INAV_OSD_LAYOUTS
refactor move all the parsing related logic to parse module in lib.rs
we can implement dump pg_group_info, which will contain array of pg_infos, so it should be faster whole group should fit there
download blackbox command, progressbar with https://github.com/mitsuhiko/indicatif
implement read from multiple files and stdin
implement interactive mode, like virtual cli optional with https://github.com/kkawakam/rustyline... maybe
implement msp message PG_DUMP_DIFF to dump only the changed diff
if we add id to msp_settings_info, we won't have to assume we receive messages in order
deal with the VAR_INT32 that eran added, its changed the api either change the api version or add the api iNav
move the parsing functions to multiwii library too, or msp lib
implemet get status
implement get tasks
make sure we are not doing unncessary clone