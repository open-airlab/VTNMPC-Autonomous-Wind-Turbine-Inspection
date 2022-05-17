$(document).ready(function () {
    // Jquery functions
    $.fn.get_id = function () {
        return $(this).attr("id")
    };
    $.fn.toggle_btn = function () {
        var el = $(this),
            disabled = el.attr("disabled")
        el.attr("disabled", !disabled)
        if (disabled) {
            el.css({
                "background-color": ''
            });
        } else {
            el.css({
                "background-color": 'gray'
            });
        }
    };
    $.fn.disable_btn = function () {
        var el = $(this)
        el.attr("disabled", true)
        el.css({
            "background-color": 'gray'
        });
    };
    $.fn.enable_btn = function () {
        var el = $(this)
        el.attr("disabled", false)
        el.css({
            "background-color": ''
        });
    };
    $.fn.return_to_state = function () {
        var state = buttons_states[$(this).get_id()]
        if (state) {
            $(this).disable_btn();
        } else {
            $(this).enable_btn();
        }
    };

    $.fn.set_btn_state = function (state) {
        buttons_states[$(this).get_id()] = state;
    };

    // ==================================================================================
    // Variables
    // ==================================================================================
    var wind_str = 0
    var wind_direction = 0
    var init_wind = false
    // Buttons
    var stop_btn = $("#stop_wind_simulation_btn")
    var start_btn = $("#start_wind_simulation_btn")
    var wd_btn = $('#wd_btn')
    var flow_btn = $('#flow_btn')
    var takeoff_btn = $('#airsim_takeoff_btn')
    var buttons_states = {}
    buttons_states[stop_btn.get_id()] = true
    buttons_states[start_btn.get_id()] = true
    buttons_states[wd_btn.get_id()] = false
    buttons_states[flow_btn.get_id()] = true
    buttons_states[takeoff_btn.get_id()] = false
    var l_buttons = [stop_btn, start_btn, wd_btn, flow_btn, takeoff_btn]


    // ==================================================================================
    // Button click test
    // ==================================================================================


    // Initialize some of the buttons:
    l_buttons.forEach(item => item.return_to_state());
    var turb_img_utf8 = "data:image/png;base64,"

    // Time constrained calls:
    function update_canvas() {
        $.getJSON('/inspection/drone_canvas_pos', {},
            function (data) {
                console.log("request_drone_data");
                console.log(data);
                $('#drone').css({
                    'left': data['responds'][0],
                    'top': data['responds'][1]
                });
            });
        $.getJSON('/inspection/get_wind_info', {},
            function (data) {
                $('#wind_info_text').html("wind speed: " + data["responds"]["ws"] + ", wind direction: " + data["responds"]["wd"] + ", ti: " + data["responds"]["wti"]);
                var img_utf8 = "data:image/png;base64," + data["responds"]["turb_img"]
                if (img_utf8 !== turb_img_utf8) {
                    turb_img_utf8 = img_utf8
                    $('#turb_img').attr("src", img_utf8);
                }
                $('#turb_smoothness').html(data["responds"]["wturb"])
            });
    }

    setInterval(update_canvas, 500)

    // Get the button that opens the modal
    var btn = document.getElementById("test_button");
    console.log(btn);
    // When the user clicks on the button, open the modal
    btn.onclick = function () {
        $.getJSON('/inspection/mission_info', {},
            function (data) {
                console.log(data);
            });
        return false;
    }
    $('.test_button').on('click', function (e) {
        var status = $(this).attr('id'); // Get id of click item.
        console.log(status)
    });
    toggle_wind_wheel = function (e) {
        var rw = $('#rotation_wheel')
        var state = rw.css('display')
        if (state === "none") {
            state = 'block';
        } else {
            state = 'none';
        }
        rw.css({'display': state})
    };
    wd_btn.on('click', toggle_wind_wheel);

    flow_btn.on('click', function (e) {
        l_buttons.forEach(item => item.disable_btn());
        $.getJSON('/inspection/flow_map', {},
            function (data) {
                $('#flow_map_img').attr("src", "data:image/png;base64," + data["flow_map"])
                $('#ti_map_img').attr("src", "data:image/png;base64," + data["ti_map"])
                $('.canvas').css({
                    'background-img': 'none',
                    'background': 'url(data:image/png;base64,' + data["flow_data_blue"] + ')'
                })
                console.log(data);
                start_btn.set_btn_state(false)
                l_buttons.forEach(item => item.return_to_state());
            });
    });

    takeoff_btn.on('click', function (e) {
        $.getJSON('/inspection/airsim_takeoff', {},
            function (data) {
                console.log("Takeoff: ", data);
            });
    });

    $('.wind-turbine').on('click', function (e) {
        var id_btn = $(this).attr('id'); // Get id of click item.
        var id_modal = "Modal" + id_btn.replace("_btn", '')
        var modal_wd_text = $('#' + id_modal + " .wd")
        var modal_wp_text = $('#' + id_modal + " .wp")
        modal_wd_text.html(`Direction = ${wind_direction}`)
        modal_wp_text.html(`Power = ${wind_str} m/s`)
    });

    start_btn.on('click', function (e) {
        $.getJSON('/inspection/start_wind_sim', {},
            function (data) {
                console.log("Takeoff: ", data);
                if (data["responds"] === 200) {
                    flow_btn.set_btn_state(true);
                    wd_btn.set_btn_state(true);
                    start_btn.set_btn_state(true)
                    stop_btn.set_btn_state(false);
                    l_buttons.forEach(item => item.return_to_state());

                    start_btn.css({
                        'display': "none"
                    });
                    stop_btn.css({
                        'display': "inline-block"
                    });
                }

            });
    });
    stop_btn.on('click', function (e) {
        $.getJSON('/inspection/stop_wind_sim', {},
            function (data) {
                console.log("Takeoff: ", data);
                if (data["responds"] === 200) {
                    flow_btn.set_btn_state(false);
                    wd_btn.set_btn_state(false);
                    start_btn.set_btn_state(false);
                    stop_btn.set_btn_state(true);

                    l_buttons.forEach(item => item.return_to_state());
                    stop_btn.css({
                        'display': "none"
                    });
                    start_btn.css({
                        'display': "inline-block"
                    });
                }
            });
    });

    // ==================================================================================
    // Rotate wind
    // ==================================================================================
    // Jquery function for turning matrix into deg and rad
    $.fn.rotationInfo = function () {
        var el = $(this),
            tr = el.css("-webkit-transform") || el.css("-moz-transform") || el.css("-ms-transform") || el.css("-o-transform") || '',
            info = {rad: 0, deg: 0};
        if (tr = tr.match('matrix\\((.*)\\)')) {
            tr = tr[1].split(',');
            if (typeof tr[0] != 'undefined' && typeof tr[1] != 'undefined') {
                info.rad = Math.atan2(tr[1], tr[0]);
                info.deg = parseFloat((info.rad * 180 / Math.PI).toFixed(1));
            }
        }
        return info;
    };
    $('#rotateable').draggable({
        opacity: 0.001,
        helper: 'clone',
        start: function (event) {
            $('#debug_center').css({
                'display': "block"
            });
            $('#debug_click').css({
                'display': "block"
            });
        },
        drag: function (event) {
            // Compute center
            var pw = document.getElementById('rotateable')
            var pwBox = pw.getBoundingClientRect()
            var center_x = (pwBox.left + pwBox.right) / 2
            var center_y = (pwBox.top + pwBox.bottom) / 2
            $('#debug_center').css({
                'margin-left': center_x - 12.5,
                'margin-top': center_y - 12.5
            })
            // get mouse position
            var mouse_x = event.pageX
            var mouse_y = event.pageY
            $('#debug_click').css({
                'margin-left': mouse_x - 12.5,
                'margin-top': mouse_y - 12.5
            })
            var dist = Math.sqrt(Math.pow(center_x - mouse_x, 2) + Math.pow(center_y - mouse_y, 2))
            wind_str = dist / 30
            wind_str = Math.round(wind_str * 10) / 10

            // console.log("wind_str: ", wind_str)

            var radians = Math.atan2(mouse_x - center_x, mouse_y - center_y)
            var degree = Math.round((radians * (180 / Math.PI) * -1) + 180);
            wind_direction = degree
            var rotateCSS = 'rotate(' + (degree) + 'deg)';
            $('#wind_direction_text').html(`Direction: ${wind_direction}, Power: ${wind_str} m/s`)
            // $('#wind_direction_text').html(degree)
            $('#rotateable').css({
                '-moz-transform': rotateCSS,
                '-webkit-transform': rotateCSS
            });
        },
        stop: function (event) {
            // var degree = $('#rotateable').rotationInfo().deg
            $.post('/inspection/set_wind_direction', {'direction': wind_direction, 'power': wind_str},
                function (data) {
                    if (data["reponds"] === 200) {
                        console.log("SUCCESS");

                    }
                    console.log(data);
                });
            $('#debug_center').css({
                'display': "none"
            });
            $('#debug_click').css({
                'display': "none"
            });
            console.log(`sending to server: \nwind_direction: ${wind_direction},\n wind_str ${wind_str}`)
            toggle_wind_wheel()
            start_btn.set_btn_state(true)
            if (!init_wind) {
                init_wind = true;
                flow_btn.set_btn_state(false);
            }
            l_buttons.forEach(item => item.return_to_state());

        }
    });

    // $('#turb_smooth_slider').slider().on('slide', function (event) {
    //     var a = event.value.newValue;
    //     var b = event.value.oldValue;
    //
    //     var changed = !($.inArray(a[0], b) !== -1 &&
    //         $.inArray(a[1], b) !== -1 &&
    //         $.inArray(b[0], a) !== -1 &&
    //         $.inArray(b[1], a) !== -1 &&
    //         a.length === b.length);
    //     if (changed) {
    //         $.post('/inspection/set_turb_scale', {'scalar': a},
    //             function (data) {
    //                 if (data["reponds"] === 200) {
    //                     console.log("SUCCESS");
    //                 }
    //                 console.log(data);
    //             });
    //     }
    // });
    const turb_slider = $("#turb_smooth_slider");
    const turb_bubble = $("#turb_smooth_bubble");
    turb_slider.change(function () {
        $.post('/inspection/set_turb_info', {'scalar': turb_slider.val()},
            function (data) {
                if (data["reponds"] === 200) {
                    console.log("SUCCESS");
                }
                console.log(data);
            });
        setBubble(turb_slider, turb_bubble);
    });

    const turb_mag_slider = $("#turb_mag_slider");
    const turb_mag_bubble = $("#turb_mag_bubble");
    turb_mag_slider.change(function () {
        $.post('/inspection/set_turb_info', {'mag': turb_mag_slider.val()},
            function (data) {
                if (data["reponds"] === 200) {
                    console.log("SUCCESS");
                }
                console.log(data);
            });
        setBubble(turb_mag_slider, turb_mag_bubble);
    });

    function setBubble(range, bubble) {
        const val = range.val();
        const min = range.attr('min') ? range.attr('min') : 0;
        const max = range.attr('max') ? range.attr('max') : 100;
        const newVal = Number(((val - min) * 100) / (max - min));
        bubble.html(val);
        // Sorta magic numbers based on size of the native UI thumb
        bubble.css({
            'left': `calc(${newVal}% + (${8 - newVal * 0.17}px))`
        });
    }

    setBubble(turb_slider, turb_bubble);
    setBubble(turb_mag_slider, turb_mag_bubble);
    // function update_canvas() {
    //     // your function code here
    //     // $.getJSON('/inspection/airsim_takeoff', {},
    //     //     function (data) {
    //     //         console.log("Takeoff: ", data);
    //     //     });
    //     setTimeout(update_canvas, 2000);
    // }
    //
    // update_canvas();

    // ==================================================================================
    // Scroll function
    // ==================================================================================
    $(function () {
        $.scrollify({
            section: ".panel",
        });
    });
});