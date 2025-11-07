import flet as ft
import serial
import serial.tools.list_ports
import time

PREFIX = 'lx-csl '
DISCOVERY = "discovery"
CONFIG = "config"
CONFIG_OK = "OK"
CONFIG_ERR = "ERR"
END_MESSAGE = "\n"


def main(page: ft.Page):
    page.title = "MIDI Console Configurator"
    page.vertical_alignment = ft.MainAxisAlignment.START
    page.scroll = ft.ScrollMode.AUTO
    page.padding = 20

    # Get available serial ports
    def get_serial_ports():
        ports = serial.tools.list_ports.comports()
        return [(p.device, p.description) for p in ports]

    # Parse discovery response
    def parse_discovery_response(response):
        """Parse the discovery response and return channel configuration"""
        # Remove prefix if present
        if response.startswith(PREFIX + DISCOVERY):
            # Remove beggingin of the command
            response = response[len(PREFIX + DISCOVERY):]

        analog_channels = []
        digital_channels = []
        analog_count = 0
        digital_count = 0

        # Split by semicolon
        parts = response.split(';')

        for part in parts:
            part = part.strip()
            if part.startswith('a:'):
                analog_count = int(part[2:])
            elif part.startswith('d:'):
                digital_count = int(part[2:])
            elif part.startswith('ac:'):
                # Analog channel: ac:primary,secondary
                values = part[3:].split(',')
                if len(values) == 2:
                    analog_channels.append({
                        'primary': int(values[0]),
                        'secondary': int(values[1])
                    })
            elif part.startswith('dc:'):
                # Digital channel: dc:cc_number
                digital_channels.append({
                    'cc': int(part[3:])
                })

        return {
            'analog_count': analog_count,
            'digital_count': digital_count,
            'analog_channels': analog_channels,
            'digital_channels': digital_channels
        }

    # Create channel UI
    def create_channel_ui(config):
        """Create UI elements for all channels"""
        channels_container = ft.Column(spacing=20)

        if config['analog_channels']:
            # Analog channels section
            analog_title = ft.Text(
                f"🔘 Analog Channels ({len(config['analog_channels'])})",
                size=20,
                weight=ft.FontWeight.BOLD,
                color="blue"
            )
            channels_container.controls.append(analog_title)

            # Create analog channels in rows
            analog_row = ft.Row(wrap=True, spacing=10)
            for i, channel in enumerate(config['analog_channels']):
                channel_card = create_analog_channel_card(i, channel)
                analog_row.controls.append(channel_card)
            channels_container.controls.append(analog_row)

        if config['digital_channels']:
            # Digital channels section
            digital_title = ft.Text(
                f"🔘 Digital Channels ({len(config['digital_channels'])})",
                size=20,
                weight=ft.FontWeight.BOLD,
                color="green"
            )
            channels_container.controls.append(digital_title)

            # Create digital channels in rows
            digital_row = ft.Row(wrap=True, spacing=10)
            for i, channel in enumerate(config['digital_channels']):
                channel_card = create_digital_channel_card(i, channel)
                digital_row.controls.append(channel_card)
            channels_container.controls.append(digital_row)

        return channels_container

    def create_analog_channel_card(index, channel):
        """Create a card for an analog channel"""
        def validate_midi_cc(e):
            """Validate MIDI CC input (0-127)"""
            try:
                value = e.control.value
                if value == "":
                    return
                num_value = int(value)
                if num_value < 0:
                    e.control.value = "0"
                elif num_value > 127:
                    e.control.value = "127"
                e.control.update()
            except ValueError:
                # Remove non-numeric characters
                e.control.value = ''.join(filter(str.isdigit, e.control.value))
                e.control.update()

        primary_field = ft.TextField(
            label="Primary MIDI CC",
            value=str(channel['primary']),
            width=150,
            text_align=ft.TextAlign.CENTER,
            on_change=validate_midi_cc,
            input_filter=ft.NumbersOnlyInputFilter()
        )

        secondary_field = ft.TextField(
            label="Secondary MIDI CC",
            value=str(channel['secondary']),
            width=150,
            text_align=ft.TextAlign.CENTER,
            on_change=validate_midi_cc,
            input_filter=ft.NumbersOnlyInputFilter()
        )

        # Store references to the fields for later access
        primary_field.data = f"analog_{index}_primary"
        secondary_field.data = f"analog_{index}_secondary"

        card = ft.Card(
            content=ft.Container(
                content=ft.Column([
                    ft.Text(
                        f"Analog {index + 1}", weight=ft.FontWeight.BOLD, text_align=ft.TextAlign.CENTER),
                    primary_field,
                    secondary_field
                ], horizontal_alignment=ft.CrossAxisAlignment.CENTER),
                padding=15,
                width=180
            ),
            elevation=2
        )

        return card

    def create_digital_channel_card(index, channel):
        """Create a card for a digital channel"""
        def validate_midi_cc(e):
            """Validate MIDI CC input (0-127)"""
            try:
                value = e.control.value
                if value == "":
                    return
                num_value = int(value)
                if num_value < 0:
                    e.control.value = "0"
                elif num_value > 127:
                    e.control.value = "127"
                e.control.update()
            except ValueError:
                # Remove non-numeric characters
                e.control.value = ''.join(filter(str.isdigit, e.control.value))
                e.control.update()

        cc_field = ft.TextField(
            label="MIDI CC",
            value=str(channel['cc']),
            width=150,
            text_align=ft.TextAlign.CENTER,
            on_change=validate_midi_cc,
            input_filter=ft.NumbersOnlyInputFilter()
        )

        # Store reference to the field for later access
        cc_field.data = f"digital_{index}_cc"

        card = ft.Card(
            content=ft.Container(
                content=ft.Column([
                    ft.Text(
                        f"Digital {index + 1}", weight=ft.FontWeight.BOLD, text_align=ft.TextAlign.CENTER),
                    cc_field
                ], horizontal_alignment=ft.CrossAxisAlignment.CENTER),
                padding=15,
                width=180
            ),
            elevation=2
        )

        return card

    port_list = get_serial_ports()

    # UI elements
    logo_image = ft.Image(
        src="HQ2.png",
        width=60,
        height=60,
        fit=ft.ImageFit.CONTAIN
    )

    title_text = ft.Text("MIDI USB Console Configurator",
                         size=24, weight=ft.FontWeight.BOLD)

    title_row = ft.Row([
        logo_image,
        title_text
    ], alignment=ft.MainAxisAlignment.START, spacing=15)
    port_dropdown = ft.Dropdown(
        label="Select a Serial Port",
        options=[ft.dropdown.Option(
            key=port, text=f"{port} - {desc}") for port, desc in port_list],
        width=400,
    )
    status_text = ft.Text(value="Not connected", color="red")

    # Container for dynamic channel configuration
    channels_container = ft.Container()

    # Send config button (initially hidden)
    send_config_button = ft.ElevatedButton(
        "Send Config",
        on_click=None,  # Will be set later
        visible=False
    )

    # Config feedback text (initially hidden)
    config_feedback_text = ft.Text(
        value="",
        visible=False
    )

    def refresh_ports(e):
        port_list = get_serial_ports()
        port_dropdown.options = [
            ft.dropdown.Option(key=port, text=f"{port} - {desc}") for port, desc in port_list]
        page.update()

    def connect_serial(e):
        if not port_dropdown.value:
            status_text.value = "⚠️ Please select a port first!"
            status_text.color = "orange"
        else:
            try:
                ser = serial.Serial(port_dropdown.value, 115200, timeout=1)
                status_text.value = f"✅ Connected to {port_dropdown.value}"
                status_text.color = "green"
                page.session.set("serial_connection", ser)
                # Send discovery command and read response
                discovery_message = PREFIX + DISCOVERY + END_MESSAGE
                ser.write(discovery_message.encode())
                # Wait a moment for response
                time.sleep(0.1)
                response = ser.readline().decode(errors="ignore").strip()

                if response and response.startswith(PREFIX + DISCOVERY):
                    try:
                        config = parse_discovery_response(response)
                        channels_ui = create_channel_ui(config)
                        channels_container.content = channels_ui
                        page.session.set("channel_config", config)
                        send_config_button.visible = True
                        config_feedback_text.visible = True
                        status_text.value += f" - Found {len(config['analog_channels'])} analog, {len(config['digital_channels'])} digital channels"
                    except Exception as parse_err:
                        status_text.value = f"❌ Parse error: {parse_err}"
                        status_text.color = "red"
                        ser.close()
                        page.session.set("serial_connection", None)
                        send_config_button.visible = False
                        config_feedback_text.visible = False
                        channels_container.content = None
                else:
                    # Invalid or no response from device
                    if response:
                        status_text.value = f"❌ Invalid response from device: '{response}' - Expected '{PREFIX + DISCOVERY} ...'"
                    else:
                        status_text.value = "❌ No response from device - Device may not be compatible"
                    status_text.color = "red"
                    ser.close()
                    page.session.set("serial_connection", None)
                    send_config_button.visible = False
                    config_feedback_text.visible = False
                    channels_container.content = None

            except Exception as err:
                status_text.value = f"❌ Error: {err}"
                status_text.color = "red"
                # Ensure we clean up if there was a connection
                ser = page.session.get("serial_connection")
                if ser:
                    try:
                        ser.close()
                    except:
                        pass
                    page.session.set("serial_connection", None)
                send_config_button.visible = False
                config_feedback_text.visible = False
                channels_container.content = None
        page.update()

    def send_config(e):
        """Send the updated configuration to the device"""
        ser = page.session.get("serial_connection")
        if ser is None:
            config_feedback_text.value = "❌ Not connected!"
            config_feedback_text.color = "red"
            page.update()
            return

        try:
            # Collect current values from all text fields
            analog_channels = []
            digital_channels = []

            # Find all text fields in the channels container
            def find_text_fields(control, fields_list):
                if isinstance(control, ft.TextField) and hasattr(control, 'data') and control.data:
                    fields_list.append(control)
                elif hasattr(control, 'controls') and control.controls:
                    for child in control.controls:
                        find_text_fields(child, fields_list)
                elif hasattr(control, 'content') and control.content:
                    find_text_fields(control.content, fields_list)

            text_fields = []
            if channels_container.content:
                find_text_fields(channels_container.content, text_fields)

            # Parse analog channels
            analog_pairs = {}
            for field in text_fields:
                if field.data and field.data.startswith('analog_'):
                    parts = field.data.split('_')
                    if len(parts) == 3:
                        index = int(parts[1])
                        field_type = parts[2]
                        if index not in analog_pairs:
                            analog_pairs[index] = {}
                        analog_pairs[index][field_type] = field.value or "0"

            # Convert to analog channels list
            for i in sorted(analog_pairs.keys()):
                pair = analog_pairs[i]
                analog_channels.append(
                    f"ac:{pair.get('primary', '0')},{pair.get('secondary', '0')}")

            # Parse digital channels
            for field in text_fields:
                if field.data and field.data.startswith('digital_'):
                    cc_value = field.value or "0"
                    digital_channels.append(f"dc:{cc_value}")

            # Build config message
            config_parts = []
            config_parts.append(f"a:{len(analog_channels)}")
            config_parts.append(f"d:{len(digital_channels)}")
            config_parts.extend(analog_channels)
            config_parts.extend(digital_channels)

            config_message = PREFIX + CONFIG + " " + \
                ';'.join(config_parts) + END_MESSAGE

            # Send the config
            ser.write(config_message.encode())
            config_feedback_text.value = f"🕐 Sending config: {len(analog_channels)} analog, {len(digital_channels)} digital channels..."
            config_feedback_text.color = "blue"
            page.update()

            # Wait for device response
            import time
            time.sleep(0.1)
            response = ser.readline().decode(errors="ignore").strip()

            # Handle device response
            if response == PREFIX + CONFIG + " " + CONFIG_OK:
                config_feedback_text.value = f"✅ Configuration applied successfully! ({len(analog_channels)} analog, {len(digital_channels)} digital channels)"
                config_feedback_text.color = "green"
            elif response == PREFIX + CONFIG + " " + CONFIG_ERR:
                config_feedback_text.value = f"❌ Device rejected configuration - Please check your settings and try again"
                config_feedback_text.color = "red"
            elif response.startswith(PREFIX + CONFIG):
                # Unexpected config response
                config_feedback_text.value = f"⚠️ Unexpected device response: '{response}' - Expected '{CONFIG_OK}' or '{CONFIG_ERR}'"
                config_feedback_text.color = "orange"
            elif response:
                # Some other response from device
                config_feedback_text.value = f"❌ Invalid response from device: '{response}' - Expected '{PREFIX + CONFIG + ' ' + CONFIG_OK}' or '{PREFIX + CONFIG + ' ' + CONFIG_ERR}'"
                config_feedback_text.color = "red"
            else:
                # No response from device
                config_feedback_text.value = f"❌ No response from device - Configuration may not have been applied"
                config_feedback_text.color = "red"

        except Exception as err:
            config_feedback_text.value = f"❌ Send error: {err}"
            config_feedback_text.color = "red"
            import traceback
            print("Error details:", traceback.format_exc())

        page.update()

    # Set the send config button callback
    send_config_button.on_click = send_config

    connect_button = ft.ElevatedButton(
        "Connect & Discover", on_click=connect_serial)
    refresh_button = ft.IconButton(
        icon=ft.Icons.REFRESH, tooltip="Refresh ports", on_click=refresh_ports)

    # Layout
    page.add(
        title_row,
        ft.Row([port_dropdown, refresh_button]),
        connect_button,
        status_text,
        ft.Divider(),
        channels_container,
        ft.Divider(),
        ft.Row([send_config_button, config_feedback_text], spacing=20),
    )


ft.app(target=main)
