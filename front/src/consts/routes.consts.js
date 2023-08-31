import {IconAntennaBars5, IconHome, IconLogout, IconPlus, IconRobot} from "@tabler/icons-react";

export const ROUTER_LINKS = [
	{ link: '/', label: 'Dashbooard', icon: IconHome },
	{ link: '/CreateTask', label: 'Create Task', icon: IconPlus },
	{ link: '/Telemetris', label: 'Telemetris', icon: IconAntennaBars5 },
	{ link: '/Robot', label: 'Robot', icon: IconRobot }
];

export const FOOTER_LINKS = [
	{ link: '/Logout', label: 'Logout', icon: IconLogout },
]
