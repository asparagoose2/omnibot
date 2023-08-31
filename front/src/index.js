import React from 'react';
import ReactDOM from 'react-dom/client';
import './index.css';
import {createBrowserRouter, RouterProvider} from "react-router-dom";
import ModelsPreview from './pages/ModelsPreview/ModelsPreview';
import Layout from './components/Layout/Layout';
import {MantineProvider} from '@mantine/core';
import {Provider} from 'react-redux'
import {store} from './store/store';
import CreateTask from './pages/CreateTask/CreateTask';
import Telemetris from "./pages/Telemetris/Telemetris";
import Robot from "./pages/Robot/Robot";

const root = ReactDOM.createRoot(document.getElementById('root'));


const router = createBrowserRouter([
	{
		path: "/",
		element: (
			<Layout><ModelsPreview/></Layout>
		),
	},
	{
		path: "Telemetris",
		element: <Layout><Telemetris/></Layout>
	},
	{
		path: "CreateTask",
		element: <Layout><CreateTask/></Layout>
	},
	{
		path: "Robot",
		element: <Layout><Robot/></Layout>
	},
]);

root.render(
	<React.StrictMode>
		<Provider store={store}>
			<MantineProvider
				withGlobalStyles
				withNormalizeCSS
				theme={{
					primaryColor: 'blue',
					colorScheme: 'dark' 
				}}
			>
				<RouterProvider router={router}/>
			</MantineProvider>
		</Provider>
	</React.StrictMode>
);
