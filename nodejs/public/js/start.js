'use strict';

//const vConsole = new VConsole();
//window.datgui = new dat.GUI();

var base_url = "yNode.jsƒT[ƒo‚ÌURLz";

var vue_options = {
    el: "#top",
    mixins: [mixins_bootstrap],
    data: {
        device_list: [],
    },
    computed: {
    },
    methods: {
        toDatetimeString: function(tim){
            return new Date(tim).toLocaleString();
        },
        toDeviceType: function(id){
            switch(id){
                case 1: return "Stick-N-Find";
                case 2: return "Project Linking";
                case 3: return "LINE Things";
                case 4: return "Immediate Alert";
                default: return "Unknown";
            }
        },
        toAddressType: function(id){
            switch(id){
                case 0x00: return "Public";
                case 0x01: return "Random";
                case 0x02: return "RPA Public";
                case 0x03: return "RPA Random";
                default: return "Unknown";
            }
        },
        list_refresh: async function(){
            var result = await do_post(base_url + "/bledevice-list" );
            this.device_list = result.list;
        }
    },
    created: function(){
    },
    mounted: function(){
        proc_load();

        this.list_refresh();
        setInterval(() =>{
            this.list_refresh();
        }, 10000);
    }
};
vue_add_data(vue_options, { progress_title: '' }); // for progress-dialog
vue_add_global_components(components_bootstrap);
vue_add_global_components(components_utils);

/* add additional components */
  
window.vue = new Vue( vue_options );
