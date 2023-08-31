import React from 'react';
import {ActionIcon, AppShell, Avatar, createStyles, Flex, Header, Navbar, rem, Title} from '@mantine/core';
import Link from '../Link/Link';
import { useLocation } from 'react-router-dom';
import {FOOTER_LINKS, ROUTER_LINKS} from "../../consts/routes.consts";
import {IconBellRinging} from "@tabler/icons-react";
import { Controller } from '../../pages/Robot/components';

const useStyles = createStyles((theme) => ({
	footer: {
		paddingTop: theme.spacing.md,
		marginTop: theme.spacing.md,
		borderTop: `${rem(1)} solid ${
			theme.colorScheme === 'dark' ? theme.colors.dark[4] : theme.colors.gray[2]
		}`,
	}
}));



const Layout = ({children}) => {
	const { classes } = useStyles();
	const location = useLocation();
	const links = ROUTER_LINKS.map((item) => <Link key={item.label} item={item} currentLocation={location.pathname} />);

	return (
		<AppShell
			padding="md"
			navbar={
			<Navbar width={{ base: 300 }}
					style={{display: 'flex', flexDirection: 'column', justifyContent: 'space-between'}}
					p="xs">
				<div>
					{links}
				</div>
				<Navbar.Section className={classes.footer}>
					{FOOTER_LINKS.map((item) => <Link key={item.label} item={item} currentLocation={location.pathname} />)}
				</Navbar.Section>
			</Navbar>
		}
			header={
				<Header height="3rem">
					<Flex justify="space-between" align="center" px="1rem">
						<Title>Omnibot</Title>
						<Flex justify="space-between" align="center">
						<Controller/>
							<Avatar radius="1rem"/>
							<ActionIcon size="lg" radius="xl" variant="light" ml={7}>
								<IconBellRinging size="1.625rem" />
							</ActionIcon>
						</Flex>
					</Flex>
				</Header>
			}
			styles={(theme) => ({
				main: { backgroundColor: theme.colorScheme === 'dark' ? theme.colors.dark[8] : theme.colors.gray[0] },
			})}
		>
			{children}
		</AppShell>
	);
};

export default Layout;
